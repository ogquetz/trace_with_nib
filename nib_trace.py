#!/usr/bin/env python
# coding=utf-8
#
# Copyright (C) 2022 Peter Garza, og.quetz@gmail.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
"""
Draw with a nib over a selected path. Heavily influenced/learned/borrowed from motion.py extension.
"""
import math

import inkex
from inkex import PathElement
from inkex.paths import Move, Line, Curve, ZoneClose, Arc, Path, Vert, Horz, TepidQuadratic, Quadratic, Smooth
from inkex.transforms import Vector2d
from inkex.bezier import beziertatslope, beziersplitatt

class PathSplitter():
    """Convenience class to split path segments"""

    def __init__(self, path, angle):
        super(PathSplitter, self).__init__()
        self.path_element = path
        self.path = path.path
        self.angle = angle
        self.info = ''

        self.split_path = Path(self.process_path())

    def process_command (self, seg, angle):
        segments = []
        d = Vector2d.from_polar(1.0, math.radians(angle))

        # Code taken from motion.py extension
        if isinstance(seg.command, (Curve, Smooth, TepidQuadratic, Quadratic, Arc)):
            prev = seg.previous_end_point
            for curve in seg.to_curves():
                bez = [prev] + curve.to_bez()
                prev = curve.end_point(seg.first_point, prev)
                tees = [t for t in beziertatslope(bez, d) if 0 < t < 1]
                tees.sort()
                if len(tees) == 1:
                    one, two = beziersplitatt(bez, tees[0])
                    segments.append(Curve(*(one[1] + one[2] + one[3])))
                    segments.append(Curve(*(two[1] + two[2] + two[3])))
                elif len(tees) == 2:
                    one, two = beziersplitatt(bez, tees[0])
                    two, three = beziersplitatt(two, tees[1])
                    segments.append(Curve(*(one[1] + one[2] + one[3])))
                    segments.append(Curve(*(two[1] + two[2] + two[3])))
                    segments.append(Curve(*(three[1] + three[2] + three[3])))
                else:
                    segments.append(curve)
        elif isinstance(seg.command, (Line, Move)):
            segments.append(seg.command)
        elif isinstance(seg.command, ZoneClose):
            """
            WARNING: Paths with multiple z commands will be incorrect.
            For some reason, the ZoneClose Path Command always returns the start
            point of the entire path (not just the command segment) for both
            ZoneClose.end_point and first_point. So for paths with multiple
            segments, all ZoneClose commands will not connect to the start of the
            path, instead of connecting to their respective segment starting point.
            """
            segments.append(Line(seg.end_point.x, seg.end_point.y))
        elif isinstance(seg.command, (Vert, Horz)):
            segments.append(seg.command.to_line(seg.end_point))

        return segments

    def process_path(self):
        path_temp = self.path
        angles = [self.angle, self.angle + 90, 0, 90]
        for angle in angles:
            # split path segments at tangents to angles
            seg_list = []
            for node in path_temp.to_absolute().proxy_iterator():
                seg_list.append(self.process_command(node, angle))

            # list comprehension to flatten command list
            result = [cmd for seg_cmd in seg_list for cmd in seg_cmd]
            path_temp = Path(result)

        return result


    def get_path_element(self):
        elem = inkex.PathElement()

        path_d = self.process_path()
        elem.path = Path(path_d)
        elem.set('inkscape:label', 'split_path')
        elem.style = self.path.style

        return elem


class NibOutline(inkex.EffectExtension):
    def add_arguments(self, pars):
        pars.add_argument("--nib_angle", type=float, help="Angle to rotate the nib")

    def effect(self):
        # TODO: update paint_order() to rendering_order() when API v1.2 is supported in macOS build. Currently using Inkscape 1.1.2 (b8e25be8, 2022-02-05)
        args = self.svg.selection.paint_order()
        if len(args) != 2:
            inkex.errormsg(_("This extension requires two selected paths."))
            raise inkex.utils.AbortExtension

        layer = self.svg.get_current_layer()
        glyph = layer.add(inkex.Layer())
        glyph.set('inkscape:label', 'glyph')

        """
        Prepare trace paths
        Uses bottom object in selection as trace path. Only paths or groups of paths are supported.
        """
        trace = args[0]
        if trace.transform:
            glyph.transform = trace.transform
            trace.transform = None

        trace = self.prepare_path(args[0], glyph, 'trace')

        """
        Prepare nib paths
        Uses top object in selection as nib path. Only paths or groups of paths are supported.
        """
        nib = self.prepare_path(args[1], glyph, 'nib')

        """
        # Place duplicates nibs at trace path end_points
        """
        for trace_cmd in trace.path.to_absolute().proxy_iterator():
            joint = self.place_joint(nib, trace_cmd.end_point, glyph)
            if isinstance(trace_cmd.command, Move): continue
            else:
                # Join end_points of current and previous joints with edge
                for nib_cmd in joint.path.to_absolute().proxy_iterator():
                    if isinstance(nib_cmd.command, (Move)): continue
                    else:
                        self.make_face(trace_cmd,
                                       nib_cmd,
                                       glyph,
                                       nib.style)

        # Cleanup:
        # Delete nib and edge paths
        trace.delete()
        nib.delete()

        # TODO: auto-select generated faces and joints and Union them


    def prepare_path(self, arg, layer, label='split'):
        """
        Get the descendants of top object.
        That list includes the top object as the first element, so pop it and test if it is a group.
        If it is a group, split the remaining paths. If not, assume it is a path and split it.

        # TODO: support for other shapes
        # TODO: break paths with multiple ZoneClose commands before splitting.
                See PathSplitter for warning about multiple ZoneClose
        """
        items = arg.descendants()
        item = items.pop(0)

        if isinstance(item, inkex.Group):
            paths = [path for path in items]
        else:
            paths = [item]

        result = layer.add(PathElement())
        result.set('inkscape:label', label)

        result_path = Path()
        for path in paths:
            processed_path = PathSplitter(path, self.options.nib_angle)
            result_path.append(processed_path.split_path)
            result.style = path.style

        result.path = result_path

        return result

    def place_joint(self, nib, point, glyph):
        """Place a joint path at an edge node"""
        joint = nib.duplicate()
        # joint.path = nib.split_path
        joint.set('inkscape:label', 'joint')
        glyph.append(joint)

        joint_center = joint.bounding_box().center
        tr = joint.transform
        tr.add_translate(point.x - joint_center.x, point.y - joint_center.y)
        tr.add_rotate(self.options.nib_angle, joint_center.x, joint_center.y)
        joint.apply_transform()

        return joint

    def make_face(self, edge, nib, layer, style):
        """
        edge: command proxy for trace path segment
        nib: command proxy for nib path segment
        layer: layer to attach the face
        style: style to apply to the face (default: copied from nib.style)

        Move command to first point of nib segment.
        Append original nib segment as relative.
        Append original edge segment as relative.
        Append reversed nib segment as relative.
        Append reversed edge segment as relative.

        Append path element to layer.
        Set path to Path from segments.
        """
        edge_rev = edge.reverse().to_relative(edge.end_point)
        edge_orig = edge.command.to_relative(edge.previous_end_point)

        if isinstance(nib.command, ZoneClose):
            nib_orig = Line(nib.first_point.x, nib.first_point.y).to_relative(nib_prev)
            nib_rev = Line(nib_prev.x, nib_prev.y).to_relative(nib.first_point)
        else:
            nib_orig = nib.command.to_relative(nib.previous_end_point)
            nib_rev = nib.reverse().to_relative(nib.end_point)

        segments = [Move(nib.previous_end_point.x, nib.previous_end_point.y)]
        segments.append(nib_orig)
        segments.append(edge_rev)
        segments.append(nib_rev)
        segments.append(edge_orig)
        segments.append(ZoneClose())

        face = inkex.PathElement()
        face.set('inkscape:label', 'face-' + str(nib.command))
        face.path = Path(segments)
        face.style = style

        layer.append(face)


if __name__ == '__main__':
    NibOutline().run()

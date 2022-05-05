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

        # Code taken from motion.py
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
            segments.append(Line(seg.first_point.x, seg.first_point.y))
            segments.append(seg.command)
        elif isinstance(seg.command, (Vert, Horz)):
            segments.append(seg.command.to_line(seg.end_point))

        return segments

    def process_path(self):
        path_temp = self.path
        angles = [self.angle, self.angle + 90, 0, 90]

        for angle in angles:
            #split path segments at tangents to self.angle
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

        return elem

class NibOutline(inkex.EffectExtension):
    def add_arguments(self, pars):
        pars.add_argument("--nib_angle", type=float, help="Angle to rotate the nib")

    def effect(self):
        args = self.svg.selection.paint_order().filter(inkex.PathElement)
        if len(args) != 2:
            inkex.errormsg(_("This extension requires two selected paths."))
            raise inkex.utils.AbortExtension

        trace = args[0]
        pen = args[1]

        layer = self.svg.get_current_layer()
        glyph = inkex.Group()
        glyph.set('inkscape:label', 'glyph')
        glyph.set('inkscape:groupmode', 'layer')
        layer.append(glyph)
        if trace.transform:
            glyph.transform = trace.transform
            trace.transform = None

        # Prepare trace path by splitting segments at tangents to nib_angle
        edge = PathSplitter(trace, self.options.nib_angle)
        edge_elem = glyph.add(edge.get_path_element())
        edge_elem.set('inkscape:label', 'edge')
        edge_elem.style = trace.style

        # Prepare pen path by splitting segments at tangents to nib_angle
        nib = PathSplitter(pen, self.options.nib_angle)
        nib_elem = glyph.add(nib.get_path_element())
        nib_elem.set('inkscape:label', 'nib')

        # Place duplicates nibs at trace path end_points
        for edge_cmd in edge.split_path.to_absolute().proxy_iterator():
            joint = self.place_joint(nib, edge_cmd.end_point, glyph)
            if isinstance(edge_cmd.command, Move): continue
            else:
                # Join end_points of current and previous joints with edge
                for nib_cmd in joint.path.to_absolute().proxy_iterator():
                    nib_cmd_prev = nib_cmd.previous_end_point
                    if isinstance(nib_cmd.command, Move): continue
                    else:
                        self.make_face(edge_cmd, nib_cmd, nib_cmd_prev, glyph, nib.path_element.style)

        # Cleanup: delete nib and edge paths
        # edge_elem.delete()
        # nib_elem.delete()


    def place_joint(self, nib, point, glyph):
        """Place a joint path at an edge node"""
        joint = nib.path_element.duplicate()
        joint.path = nib.split_path
        joint.set('inkscape:label', 'joint')
        glyph.append(joint)

        joint_center = joint.bounding_box().center
        tr = joint.transform
        tr.add_translate(point.x - joint_center.x, point.y - joint_center.y)
        tr.add_rotate(self.options.nib_angle, joint_center.x, joint_center.y)
        joint.apply_transform()

        return joint

    def make_face(self, edge, nib, nib_prev, layer, style):
        """
        edge: command proxy for trace path segment
        nib: command proxy for nib path segment
        nib_prev: previous end point
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
            nib_orig = inkex.paths.Line(nib.end_point.x, nib.end_point.y).to_relative(nib_prev)
            nib_rev = inkex.paths.Line(nib_prev.x, nib_prev.y).to_relative(nib.first_point)
        else:
            nib_orig = nib.command.to_relative(nib_prev)
            nib_rev = nib.reverse().to_relative(nib.end_point)

        segments = [Move(nib_prev.x, nib_prev.y)]
        segments.append(nib_orig)
        segments.append(edge_rev)
        segments.append(nib_rev)
        segments.append(edge_orig)
        segments.append(ZoneClose())

        face = inkex.PathElement()
        face.set('inkscape:label', 'face')
        face.path = Path(segments)
        face.style = style

        layer.append(face)

if __name__ == '__main__':
    NibOutline().run()

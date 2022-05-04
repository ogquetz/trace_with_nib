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



class NibOutline(inkex.EffectExtension):
    def add_arguments(self, pars):
        # pars.add_argument("--nib_height", type=float, help="Height of the nib")
        # pars.add_argument("--nib_width",  type=float, help="Width of the nib")
        pars.add_argument("--nib_angle", type=float, help="Angle to rotate the nib")

    def place_joint(self, nib, point, glyph):
        joint = nib.duplicate()
        joint.set('inkscape:label', 'joint')
        glyph.append(joint)

        joint_center = joint.bounding_box().center
        tr = joint.transform
        tr.add_translate(point.x - joint_center.x, point.y - joint_center.y)
        tr.add_rotate(self.options.nib_angle, joint_center.x, joint_center.y)
        joint.apply_transform()

        return joint

    @staticmethod
    def process_edge(cmd_proxy, group, nib_angle):
        segments = []

        d = Vector2d.from_polar(1.0, math.radians(nib_angle))

        # Code taken from motion.py
        if isinstance(cmd_proxy.command, (Curve, Smooth, TepidQuadratic, Quadratic, Arc)):
            prev = cmd_proxy.previous_end_point
            for curve in cmd_proxy.to_curves():
                bez = [prev] + curve.to_bez()
                prev = curve.end_point(cmd_proxy.first_point, prev)
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
        elif isinstance(cmd_proxy.command, (Line, Curve, Move)):
            segments.append(cmd_proxy.command)
        elif isinstance(cmd_proxy.command, ZoneClose):
            segments.append(Line(cmd_proxy.first_point.x, cmd_proxy.first_point.y))
        elif isinstance(cmd_proxy.command, (Vert, Horz)):
            segments.append(cmd_proxy.command.to_line(cmd_proxy.end_point))

        return segments

    def make_face(self, edge, nib, nib_prev, layer):
        """
        edge: command proxy for trace path segment
        nib: command proxy for nib path segment
        layer: layer to attach the face

        Move command to first point of nib segment.
        Append original nib segment as relative.
        Append original edge segment as relative.
        Append reversed nib segment as relative.
        Append reversed edge segment as relative.

        Append path element to layer.
        Set path to Path from segments.
        """
        edge_start = edge.reverse().to_relative(edge.end_point)
        edge_end = edge.command.to_relative(edge.previous_end_point)

        if isinstance(nib.command, ZoneClose):
            nib_start = inkex.paths.Line(nib.end_point.x, nib.end_point.y).to_relative(nib_prev)
            nib_end = inkex.paths.Line(nib_prev.x, nib_prev.y).to_relative(nib.first_point)
        else:
            nib_start = nib.command.to_relative(nib_prev)
            nib_end = nib.reverse().to_relative(nib.end_point)

        segments = [Move(nib_prev.x, nib_prev.y)]
        segments.append(nib_start)
        segments.append(edge_start)
        segments.append(nib_end)
        segments.append(edge_end)
        segments.append(ZoneClose())

        face = inkex.PathElement()
        face.set('inkscape:label', 'face')
        face.path = Path(segments)

        layer.append(face)


    def effect(self):
        args = self.svg.selection.paint_order().filter(inkex.PathElement)
        if len(args) != 2:
            inkex.errormsg(_("This extension requires two selected paths."))
            raise inkex.utils.AbortExtension

        trace = args[0]
        trace_path = trace.path.to_absolute()
        trace_cmd = trace_path.proxy_iterator()

        nib = args[1]

        layer = self.svg.get_current_layer()
        glyph = inkex.Group()
        glyph.set('inkscape:label', 'glyph')
        glyph.set('inkscape:groupmode', 'layer')
        layer.append(glyph)

        if trace.transform:
            glyph.transform = trace.transform
            trace.transform = None

        # Prepare trace path by splitting segments at tangents to nib_angle
        edge = inkex.PathElement()
        edge.set('inkscape:label', 'edge')
        edge.path = inkex.Path()
        edge.style = trace.style
        glyph.append(edge)

        for node in trace_cmd:
            seg = self.process_edge(node, glyph, self.options.nib_angle)
            edge.path += inkex.Path(seg)

        # Place duplicates nibs at trace path end_points
        for edge_cmd in edge.path.to_absolute().proxy_iterator():
            joint = self.place_joint(nib, edge_cmd.end_point, glyph)
            if isinstance(edge_cmd.command, Move): continue
            else:
                # Join end_points of current and previous joints with edge
                for nib_cmd in joint.path.to_absolute().proxy_iterator():
                    nib_cmd_prev = nib_cmd.previous_end_point
                    if isinstance(nib_cmd.command, Move): continue
                    else:
                        self.make_face(edge_cmd, nib_cmd, nib_cmd_prev, glyph)

if __name__ == '__main__':
    NibOutline().run()

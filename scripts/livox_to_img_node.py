#!/usr/bin/env python

##############################################################################
#
#  @file livox_to_img_node.py
#  @brief Executable ROS node for livox_to_img_node.py
#
#  Copyright (C) 2023-07-04 18:06:54.
#  Author: Iacopo Catalano
#  Email: imcata@utu.fi
#  All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
##############################################################################

import livox_to_img.livox_to_img as livox_to_img


if __name__ == "__main__":
    livox_to_img.main()
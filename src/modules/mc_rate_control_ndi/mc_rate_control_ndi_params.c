/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_rate_control_ndi_params.c
 * Parameters for multicopter attitude controller using NDI.
 *
 * @author Wenxuan Gao <jyz_john@sjtu.edu.cn>
 */

/**
 * Multicoptor's mass
 *
 * @min 0
 * @max 10.0
 * @decimal 3
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_MASS, 1.0f);

/**
 * Multicoptor's inertia about X axis
 *
 * @min 0
 * @max 1.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_IXX, 0.01f);

/**
 * Multicoptor's inertia about Y axis
 *
 * @min 0
 * @max 1.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_IYY, 0.01f);

/**
 * Multicoptor's inertia about Z axis
 *
 * @min 0
 * @max 1.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_IZZ, 0.01f);

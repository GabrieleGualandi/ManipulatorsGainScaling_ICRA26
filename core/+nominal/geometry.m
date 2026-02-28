function [ent] = geometry(J_func_task, Jdot_func_task, HT_hand_func, taskP, q, qdot)
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% J_func_task: handle to the jacobian having rows the task space
% Jdot_func_task: dJdt
% HT_hand_func: direct kinematic
% taskP: rows of the position space to be considered in the task space
 
ent.J_task          = J_func_task(q);  % Reduced Jacobian (task space rows)
ent.Jdot_task       = Jdot_func_task([q; qdot]); % Reduced dtdq Jacobian (task space rows)
HT_hand             = HT_hand_func(q);

ent.nJoints = size(ent.J_task,2);

ent.handPosition = HT_hand(1:3,4); % full size 3x1
ent.handPosition_task = ent.handPosition(taskP); % task space

ent.handOrientation = HT_hand(1:3,1:3); % full size 3x3
%ent.handOrientation_task = ent.handOrientation(taskO); % tasks space

ent.pseudoinverse_task = dampedPseudoRedundant(ent.J_task); % can use use the C++ class

ent.handGeneralizedVelocity_task = ent.J_task * qdot;

ent.P_task = eye(ent.nJoints) - ent.pseudoinverse_task * ent.J_task; % projector in the null space of J_task

ent.q = q;
ent.qdot = qdot;


end


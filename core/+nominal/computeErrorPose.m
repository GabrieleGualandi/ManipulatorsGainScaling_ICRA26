function [error_pose] = computeErrorPose(current_position_task, current_orientation_FULL, desired_position_task, desired_orientation_FULL, taskO)
    % current_position_task: with variable size (depends on the task space)
    % current_orientation_FULL: 3x3 matrix
    % desired_position_task: with variable size (depends on the task space)
    % desired_orientation_FULL: 3x3 matrix
    % taskO: vector containing the rows relative to the orientation included in the task space i.e., it contains maximum 3 elements in
    % [1,2,3].

    % Provide errors in the task space
    if ~ (size(current_orientation_FULL,1) == 3 && size(current_orientation_FULL,2) == 3 )
        error('Wrong orientation size')
    end

    %% PD Controller in the task space i.e., we control a plant with m double integrators
    
    % Value/Linear
    posit_error = desired_position_task - current_position_task;

    % Value/Angular
    rotationError_mat = desired_orientation_FULL * transpose(current_orientation_FULL);
    [axis_e, angle_e,~,~] = axisAngleInverse( rotationError_mat );
    ori_error_FULL = sin(angle_e/2)*axis_e;
    ori_error = ori_error_FULL(taskO);

    % Error at value level
    error_pose = [posit_error;ori_error];
    



    %%%
end
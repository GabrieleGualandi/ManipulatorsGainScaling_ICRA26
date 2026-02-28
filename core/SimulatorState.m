classdef SimulatorState < handle
% SPDX-License-Identifier: Apache-2.0
% Copyright (c) 2026 Gabriele Gualandi
%
% SIMULATORSTATE Encapsulates the state of the robot simulator.
% This class provides a centralized container for all simulation variables,
% including real states, estimated states, and auxiliary defence filter states.
%
% Design contract:
%   - computeInitialState(par) decides WHICH fields exist based on feature flags.
%   - initialize(obj, s) copies every field present in s onto the object.
%   - update()   copies non-empty fields from st.next back onto obj.
%   - toStruct() exports non-empty properties into a plain struct.
%
% SEE ALSO: COMPUTEINITIALSTATE, CORE_INIT, CORESIMULATOR


    properties
        %--- Always present: core robot state ---
        sys_real_q
        sys_real_qdot
        sys_observer_q_hat
        sys_observer_qdot_hat
        sys_sensingFreeProjected_q
        sys_sensingFreeProjected_qdot
        current_Pz
        y_prev_total_attack

        %--- Gain Scaling / Active Dissipation (usePDgainsScaling | useActiveDissipation) ---
        sys_gainScaling_q
        sys_gainScaling_qdot
        target_gainScaling



        %--- ActiveDissipation ---
        sys_smootDamp_q
        sys_smootDamp_qdot
        target_smootDamp

        %--- Internal containers ---
        next    % Struct holding the next-step state snapshot
        ent     % Struct holding geometric entities (temp, per-step)
    end

    methods

        function obj = SimulatorState()
            % Constructor — all properties start empty; initialize() populates them.
            obj.next = struct();
            obj.ent  = [];
        end

        function initialize(obj, s)
            % INITIALIZE  Copies every field of s onto the object.
            %
            % Input:
            %   s - Struct produced by computeInitialState(par).
            %       Only contains fields for the currently active features.
            %
            % Because we iterate over s's fields, no change is needed here
            % when features are added or removed — only computeInitialState
            % needs updating.

            fields = fieldnames(s);
            for i = 1:numel(fields)
                obj.(fields{i}) = s.(fields{i});
            end
        end

        function update(obj)
            % UPDATE  Propagates st.next -> st (current) for each field in next.
            %
            % Only fields that were actually placed in obj.next are updated,
            % so unused feature states are never touched.

            fields = fieldnames(obj.next);
            for i = 1:numel(fields)
                f = fields{i};
                obj.(f) = obj.next.(f);
            end
        end

        function s = toStruct(obj)
            % TOSTRUCT  Exports all non-empty, non-internal properties as a struct.
            %
            % 'next' and 'ent' are intentionally excluded — they are
            % transient containers, not part of the persistent state snapshot.

            excluded = {'next', 'ent'};
            s = struct();
            props = properties(obj);
            for i = 1:numel(props)
                p = props{i};
                if ~ismember(p, excluded) && ~isempty(obj.(p))
                    s.(p) = obj.(p);
                end
            end
        end

        function newObj = clone(obj)
            % CLONE  Creates a deep copy of the SimulatorState object.
            %
            % Usage: newState = st.clone();

            newObj = SimulatorState();
            newObj.initialize(obj.toStruct());

            % 'next' and 'ent' are structs — assignment is by value (deep copy).
            newObj.next = obj.next;
            newObj.ent  = obj.ent;
        end

    end
end

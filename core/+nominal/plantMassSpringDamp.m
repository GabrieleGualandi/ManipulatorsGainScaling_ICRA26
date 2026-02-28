function paramsOut = plantMassSpringDamp(paramsIn)
% Enriches the input parameters data structure with mass-spring-damping system.

    paramsOut = paramsIn; 
    meta = paramsIn.sys.meta;
    mSys = meta.mass;
    kSys = meta.spring;
    if isfield(meta,'damping')
        bSys = meta.damping;
    else 
        bSys = 2*sqrt(mSys * kSys);
    end
    
    if ~isfield(paramsOut.sys, 'info')
        paramsOut.sys.info = struct();
    end
    paramsOut.sys.info.dis.isCriticallyDamped = (bSys - (2*sqrt(mSys * kSys)))^2 < 0.00001;
    Ts = paramsIn.sys.meta.SAMPLING_TIME;

    A_c = [0 1; -kSys/mSys -bSys/mSys];
    B_c = [0 1/mSys]';
    C_c = [1 0];
    D_c = 0;

    sysCon = ss(A_c,B_c,C_c,D_c);
    paramsOut.sys.info.cont.staticGain = - C_c * (A_c \ B_c);
    [sysDis,~] = c2d(sysCon,Ts,'zoh');

    paramsOut.sys.con = sysCon;
    paramsOut.sys.dis = sysDis;
    paramsOut.sys.n  = 2;
    paramsOut.sys.m  = 1;
    paramsOut.sys.p  = 1;

    if isfield(meta,'UMAX') || isfield(meta,'UMIN')
        paramsOut.sys.UMAX  = meta.UMAX;
        paramsOut.sys.UMIN  = meta.UMIN;
    end

    paramsOut.sys.ports.controls = 1;
    paramsOut.sys.ports.known = [];
    paramsOut.sys.ports.sensors = 1;
    paramsOut.sys.kind = paramsIn.sys.meta.kind;
end

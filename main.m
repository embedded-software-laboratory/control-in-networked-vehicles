function main(varargin)
    % MAIN  main function

    vehicle_ids = cell2mat(varargin);
    Ts = 0.4; %[s]
    hlc = pmpc.HlcIdentification(Ts,vehicle_ids);
%     hlc = pmpc.Hlc(Ts,vehicle_ids);
%     hlc = cmpc.Hlc(Ts,vehicle_ids);
%     hlc = dmpc.Hlc(Ts,vehicle_ids);

    hlc.start();
end
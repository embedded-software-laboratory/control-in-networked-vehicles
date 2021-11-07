function rtigen_matlab
% rtigen_matlab converts an IDL-file to an m-file.
    setenv("LD_LIBRARY_PATH", '/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0');
    setenv("NDDSHOME", ['/', getenv('NDDSHOME')]); % Matlab is dropping the leading slash
    % import hlc idl file
    hlc_idl = '+cmmn/HlcPlan.idl';
    assert(isfile(hlc_idl));
    DDS.import(hlc_idl,'matlab', 'f');
end
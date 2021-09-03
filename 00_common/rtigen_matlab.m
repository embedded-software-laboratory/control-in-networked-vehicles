function rtigen_matlab
    setenv("LD_LIBRARY_PATH", '/opt/rti_connext_dds-6.0.0/lib/x64Linux4gcc7.3.0');
    
    % Import IDL files from cpm library
    dds_idl_matlab_dir = fullfile('../../../cpm_lib/dds_idl_matlab/');
    assert(isfolder(dds_idl_matlab_dir),...
        'Missing directory "%s".', dds_idl_matlab_dir);
    assert(~isempty(dir([dds_idl_matlab_dir, '*.m'])),...
        'No MATLAB IDL-files found in %s', dds_idl_matlab_dir);
    addpath(dds_idl_matlab_dir)

    % import hlc idl file
    hlc_idl = 'HlcPlan.idl';
    assert(isfile(hlc_idl));
    DDS.import(hlc_idl,'matlab', 'f')
end
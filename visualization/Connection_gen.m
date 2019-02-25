function obj = Connection_gen()
    dir = fileparts(mfilename('fullpath'));
    cwd = cd(dir);
    cleanup_obj = onCleanup(@() cd(cwd));
    fprintf('Compiling Connection_mex\n');
    
    mex -Isrc_cpp_for_task -I../../imex/include -I../../eigen Connection_mex.cpp ../src_cpp_for_task/Connection.cpp ../src_cpp_for_task/Spline3.cpp ../src_cpp_for_task/Manipulator.cpp ../src_cpp_for_task/Math_utils.cpp

    obj = mex_interface(str2fun([dir '/Connection_mex']));
end
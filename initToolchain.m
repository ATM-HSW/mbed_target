
% init matlab for mbed code generation
% note: run as root!

%prepares matlab path and compiles blocks
% afterwards code generation can run as normal user

setup_customtarget_mbed
cd blocks 
mex_compile_all
cd ../..
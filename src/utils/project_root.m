function root = project_root()
%PROJECT_ROOT Return the absolute project root path.
%
% 本函数位于 src/utils/project_root.m，
% 因此需要向上回退三层目录才能回到仓库根目录：
%   project_root = ../../..

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
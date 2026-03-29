function root = project_root()
%PROJECT_ROOT Return the absolute project root path.

root = fileparts(fileparts(mfilename('fullpath')));
end

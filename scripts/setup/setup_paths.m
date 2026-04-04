function setup_paths()
%SETUP_PATHS Add project folders to the MATLAB path.

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));

addpath(fullfile(projectRoot, 'src'));
addpath(genpath(fullfile(projectRoot, 'src', 'parameters')));
addpath(genpath(fullfile(projectRoot, 'src', 'environment')));
addpath(genpath(fullfile(projectRoot, 'src', 'gnc')));
addpath(genpath(fullfile(projectRoot, 'src', 'sensors')));
addpath(genpath(fullfile(projectRoot, 'src', 'observers')));
addpath(genpath(fullfile(projectRoot, 'src', 'control')));
addpath(genpath(fullfile(projectRoot, 'src', 'disturbance')));
addpath(genpath(fullfile(projectRoot, 'src', 'utils')));
addpath(genpath(fullfile(projectRoot, 'scripts', 'analysis')));
addpath(genpath(fullfile(projectRoot, 'scripts', 'visualization')));
addpath(genpath(fullfile(projectRoot, 'models', 'libraries', 'function')));

disp('Satellite project paths configured.');
end

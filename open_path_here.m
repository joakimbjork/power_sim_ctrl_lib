clear all
if(~isdeployed)
    % Change the current folder to the folder of this m-file.
    cd(fileparts(which(mfilename)));
        
    % Add that folder plus all subfolders to the path.
    addpath(genpath( fileparts(which(mfilename))));
    if isfolder('.git')
        rmpath('.git') % Exlude from path
    end
end
s = tf('s'); % Use this to check if control library is available
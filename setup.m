function setup()
    %Update new changes
    rehash;

    % Get the current directory (root of the cloned repository)
    projectRoot = pwd;

    % Add the project root to the MATLAB path
    addpath(projectRoot);
    
    % Add other necessary paths
    addpath(fullfile(projectRoot, 'eStop'));
    addpath(fullfile(projectRoot, 'images'));
    addpath(fullfile(projectRoot, 'test'));
    
    % Recursively add plyFiles and its subfolders to the MATLAB path
    addpath(genpath(fullfile(projectRoot, 'plyFiles')));

    % Save the path for future MATLAB sessions
    savepath;

    %Run the toolbox startup
    startup_rvc;

    % Confirm the setup is complete
    disp('Project setup complete. Paths have been added and toolbox initialized.');
end



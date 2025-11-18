% filepath: z:\Jassal, K\simulation.m

% Simulation of a rotating cube
% Uses quaternions for attitude parameterization
% Implements RK4 integration scheme
% Displays principal axes and instantaneous rotation axis on the cube

clear; clc; close all;

%% Parameters
% Replace the diagonal inertia with a full inertia tensor
% I = [1, 5, 1];  % Principal moments of inertia [Ixx, Iyy, Izz]

% I_tensor = [1.329 0.027 0.074;
%     0.027 1.716 -0.051;
%     0.074 -0.051 1.630].*10^(-3); 0.50.80.3

I_tensor = [5, 0, 0;    % Full inertia tensor with off-diagonal terms
           0, 5, 0;     % These represent products of inertia
           0, 0, 5];    % Non-zero off-diagonals create non-principal axis motion


w0 = [0., 0.5, 3.];  % Initial angular velocity [rad/s] x,y,z

q0 = [1, 0, 0, 0];   % Initial quaternion (scalar first) - identity

tspan = [0 10];      % Time span for simulation
dt = 0.01;           % Time step

% 1,5,3 inertial, 0.5,0,3 angular -- intermediate axis
% 0,0,3 angular -- principal axis
% 5,5,5 inertial, 0.5,0,3 angular -- precession

%% Create time 
t = tspan(1):dt:tspan(2);
n_steps = length(t);

%% Initialize state vector [q0, q1, q2, q3, wx, wy, wz]
state = zeros(n_steps, 7); %matrix of zero of n_steps by 7
state(1,:) = [q0, w0]; % initial state

%% Main simulation loop using RK4
for i = 1:n_steps-1
    k1 = derivatives(state(i,:), I_tensor);
    k2 = derivatives(state(i,:) + dt/2*k1, I_tensor);
    k3 = derivatives(state(i,:) + dt/2*k2, I_tensor);
    k4 = derivatives(state(i,:) + dt*k3, I_tensor);
    
    % Update state
    state(i+1,:) = state(i,:) + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    
    % Normalize quaternion
    q_norm = norm(state(i+1,1:4));
    state(i+1,1:4) = state(i+1,1:4) / q_norm;
end

%% Visualization setup
% Create two figures side by side
fig1 = figure('Name', 'Rotating Cube', 'units', 'normalized', 'outerposition', [0 0 0.5 1]);
h_ax = axes('Position', [0.1, 0.1, 0.8, 0.8]);

% Create second figure for live spherical representation
fig2 = figure('Name', 'Live Spherical Representation', 'units', 'normalized', 'outerposition', [0.5 0 0.5 1]);
sphere_ax = axes('Position', [0.1, 0.1, 0.8, 0.8]);

% Create unit sphere (for reference)
[X,Y,Z] = sphere(50);
surf(sphere_ax, X, Y, Z, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1, 'FaceColor', [0.8, 0.8, 0.8]);
hold(sphere_ax, 'on');
axis(sphere_ax, 'equal');
grid(sphere_ax, 'on');
xlabel(sphere_ax, 'X'); ylabel(sphere_ax, 'Y'); zlabel(sphere_ax, 'Z');
title(sphere_ax, 'Live Motion of Principal Axes on Unit Sphere', 'FontSize', 14);
view(sphere_ax, 30, 20);

% Prepare path traces on the sphere
x_path_h = plot3(sphere_ax, NaN, NaN, NaN, 'r-', 'LineWidth', 2);
y_path_h = plot3(sphere_ax, NaN, NaN, NaN, 'g-', 'LineWidth', 2);
z_path_h = plot3(sphere_ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);

% Prepare markers for current positions
x_marker_h = plot3(sphere_ax, 0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
y_marker_h = plot3(sphere_ax, 0, 0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
z_marker_h = plot3(sphere_ax, 0, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

% Add quaternion display for Figure 2
fig2_quat = text(sphere_ax, -0.9, -0.9, 0.9, 'q = [1, 0, 0, 0]', 'FontSize', 12);

% Storage for path traces
x_path_data = nan(n_steps, 3);
y_path_data = nan(n_steps, 3);
z_path_data = nan(n_steps, 3);

% Initialize lists for paths
x_list_x = []; x_list_y = []; x_list_z = [];
y_list_x = []; y_list_y = []; y_list_z = [];
z_list_x = []; z_list_y = []; z_list_z = [];

% Switch back to cube figure for setup
figure(fig1);

% Create unit cube vertices (centered at origin)
vertices = [
    -0.5, -0.5, -0.5;
    -0.5, -0.5,  0.5;
    -0.5,  0.5, -0.5;
    -0.5,  0.5,  0.5;
     0.5, -0.5, -0.5;
     0.5, -0.5,  0.5;
     0.5,  0.5, -0.5;
     0.5,  0.5,  0.5
];

% Define faces of the cube
faces = [
    1 2 4 3;  % -x face
    5 6 8 7;  % +x face
    1 2 6 5;  % -y face
    3 4 8 7;  % +y face
    1 3 7 5;  % -z face
    2 4 8 6   % +z face
];

% Face colors
face_colors = [
    0.8 0.2 0.2;  % Red
    0.8 0.2 0.2;  % Red
    0.2 0.8 0.2;  % Green
    0.2 0.8 0.2;  % Green
    0.2 0.2 0.8;  % Blue
    0.2 0.2 0.8   % Blue
];

% Initialize cube patch
cube_h = patch('Vertices', vertices, 'Faces', faces, ...
              'FaceColor', 'flat', 'FaceVertexCData', face_colors, ...
              'EdgeColor', 'k', 'LineWidth', 2);
alpha(cube_h, 0.8);  % Make cube somewhat transparent

% Principal axes lines (initially aligned with global axes)
axis_length = 1.0;  % Length of principal axis lines
axes_h = zeros(3,1);
axes_colors = {'r', 'g', 'b'};
axes_labels = {'X_p', 'Y_p', 'Z_p'};  % Principal axes labels

for i = 1:3
    % Create line for each principal axis
    xyz = zeros(2,3);
    xyz(2,i) = axis_length;  % Axis points along one of the coordinate axes
    axes_h(i) = line('XData', xyz(:,1), 'YData', xyz(:,2), 'ZData', xyz(:,3), ...
                    'Color', axes_colors{i}, 'LineWidth', 3);
    % Add label for each principal axis
    text(xyz(2,1)*1.1, xyz(2,2)*1.1, xyz(2,3)*1.1, axes_labels{i}, ...
         'Color', axes_colors{i}, 'FontSize', 14);
end

% Create rotation axis line (instantaneous axis of rotation)
rot_axis_h = line('XData', [0, 0], 'YData', [0, 0], 'ZData', [0, 0], ...
                 'Color', 'k', 'LineWidth', 4, 'LineStyle', '--');
rot_axis_label = text(0, 0, 0, 'Rot. Axis', 'Color', 'k', 'FontSize', 14);

% Create angular momentum vector line (fixed in inertial space)
ang_momentum_h = line('XData', [0, 0], 'YData', [0, 0], 'ZData', [0, 0], ...
                     'Color', 'm', 'LineWidth', 4, 'LineStyle', '-.');
ang_momentum_label = text(0, 0, 0, 'Angular Momentum', 'Color', 'm', 'FontSize', 14);

% Set axes properties
axis equal;
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Rotating Cube with Principal Axes and Instantaneous Rotation Axis');
view(30, 20);

% Animation controls
h_txt = text(-1.4, -1.4, 1.4, 'Time: 0.00 s', 'FontSize', 12);
h_quat = text(-1.4, -1.4, 1.2, 'q = [1, 0, 0, 0]', 'FontSize', 12);

% Animation
for i = 1:5:n_steps
    % Extract quaternion and convert to DCM
    q = state(i,1:4);
    w = state(i,5:7);
    dcm = quat2dcm(q);
    
    % Update cube
    figure(fig1);
    
    % Update vertices
    rotated_vertices = (dcm * vertices')';
    set(cube_h, 'Vertices', rotated_vertices);
    
    % Update principal axes
    for j = 1:3
        xyz = zeros(2,3);
        xyz(2,:) = dcm(:,j)' * axis_length;
        set(axes_h(j), 'XData', xyz(:,1), 'YData', xyz(:,2), 'ZData', xyz(:,3));
        
        % Update label positions
        text_pos = xyz(2,:) * 1.1;
        set(findobj(gca, 'String', axes_labels{j}), 'Position', text_pos);
    end
    
    % Calculate angular momentum in body frame
    w_col = w'; % Convert to column vector
    L_body = I_tensor * w_col; % Proper matrix multiplication
    L_body = L_body'; % Convert back to row vector
    
    % Transform to inertial frame - this is constant in torque-free motion
    L_inertial = (dcm * L_body')';
    
    % Normalize to get direction
    if norm(L_inertial) > 1e-6
        L_dir = L_inertial / norm(L_inertial);
    else
        L_dir = [0, 0, 1];
    end
    
    % Scale line length
    L_line = [zeros(1,3); L_dir * 1.8];
    
    % Update angular momentum line - this is the fixed rotation axis in space
    set(ang_momentum_h, 'XData', L_line(:,1), 'YData', L_line(:,2), 'ZData', L_line(:,3));
    
    % Update angular momentum label
    L_label_pos = L_dir * 1.9;
    set(ang_momentum_label, 'Position', L_label_pos);
    
    % Update instantaneous rotation axis (this changes in body frame)
    % Transform angular velocity vector from body frame to inertial frame
    w_inertial = (dcm * w')';
    
    % Normalize to get axis direction
    if norm(w_inertial) > 1e-6
        rot_axis = w_inertial / norm(w_inertial);
    else
        rot_axis = [0, 0, 1];  % Default if angular velocity is near zero
    end
    
    % Scale axis line length
    axis_line = [zeros(1,3); rot_axis * 1.5];
    
    % Update rotation axis line
    set(rot_axis_h, 'XData', axis_line(:,1), 'YData', axis_line(:,2), 'ZData', axis_line(:,3));
    
    % Update rotation axis label
    label_pos = rot_axis * 1.6;
    set(rot_axis_label, 'Position', label_pos);
    
    % Update time text
    set(h_txt, 'String', sprintf('Time: %.2f s', t(i)));
    
    % Update quaternion display
    set(h_quat, 'String', sprintf('q = [%.3f, %.3f, %.3f, %.3f]', q(1), q(2), q(3), q(4)));
    
    % Update spherical representation
    figure(fig2);
    
    % Extract current orientation of principal axes
    x_axis = dcm(:,1)';
    y_axis = dcm(:,2)';
    z_axis = dcm(:,3)';
    
    % Store the axis positions for paths
    x_list_x = [x_list_x, x_axis(1)]; %#ok<*AGROW>
    x_list_y = [x_list_y, x_axis(2)];
    x_list_z = [x_list_z, x_axis(3)]; 
    
    y_list_x = [y_list_x, y_axis(1)];
    y_list_y = [y_list_y, y_axis(2)];
    y_list_z = [y_list_z, y_axis(3)];
    
    z_list_x = [z_list_x, z_axis(1)];
    z_list_y = [z_list_y, z_axis(2)];
    z_list_z = [z_list_z, z_axis(3)];
    
    % Update paths
    set(x_path_h, 'XData', x_list_x, 'YData', x_list_y, 'ZData', x_list_z);
    set(y_path_h, 'XData', y_list_x, 'YData', y_list_y, 'ZData', y_list_z);
    set(z_path_h, 'XData', z_list_x, 'YData', z_list_y, 'ZData', z_list_z);
    
    % Update current position markers
    set(x_marker_h, 'XData', x_axis(1), 'YData', x_axis(2), 'ZData', x_axis(3));
    set(y_marker_h, 'XData', y_axis(1), 'YData', y_axis(2), 'ZData', y_axis(3));
    set(z_marker_h, 'XData', z_axis(1), 'YData', z_axis(2), 'ZData', z_axis(3));
    
    % Update angular momentum vector in sphere plot if it doesn't exist yet
    if i == 1
        quiver3(sphere_ax, 0, 0, 0, L_dir(1), L_dir(2), L_dir(3), 'k-', 'LineWidth', 3, 'MaxHeadSize', 0.5);
        text(sphere_ax, L_dir(1)*1.1, L_dir(2)*1.1, L_dir(3)*1.1, 'L', 'FontSize', 14);
        legend(sphere_ax, 'Unit sphere', 'X-axis path', 'Y-axis path', 'Z-axis path', ...
               'X-axis current', 'Y-axis current', 'Z-axis current', 'Angular momentum', ...
               'Location', 'eastoutside');
    end
    
    % Update quaternion display
    set(fig2_quat, 'String', sprintf('q = [%.3f, %.3f, %.3f, %.3f]', q(1), q(2), q(3), q(4)));
    
    drawnow;
    pause(0.01);
end

%% Store principal axes histories during animation
% Initialize arrays to store principal axes orientations
x_axis_history = zeros(n_steps, 3);
y_axis_history = zeros(n_steps, 3);
z_axis_history = zeros(n_steps, 3);

% Calculate principal axes orientation in inertial frame for all time steps
for i = 1:n_steps
    q = state(i,1:4);
    dcm = quat2dcm(q);
    
    % Each column of DCM represents one principal axis in inertial coordinates
    x_axis_history(i,:) = dcm(:,1)';
    y_axis_history(i,:) = dcm(:,2)';
    z_axis_history(i,:) = dcm(:,3)';
end

% Calculate angular momentum (constant in inertial space)
w_init = state(1,5:7)';  % Column vector
L_body = I_tensor * w_init;  % Proper matrix multiplication
dcm_init = quat2dcm(state(1,1:4));
L_inertial = dcm_init * L_body;  % Result is column vector
L_dir = L_inertial / norm(L_inertial);

%% Plotting the motion of the principal axes on a sphere with a time slider
sphere_fig = figure('Name', 'Spherical Representation with Time Control', 'Position', [300, 300, 950, 800]);

% Create a unit sphere (for reference)
[X,Y,Z] = sphere(50);
surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1, 'FaceColor', [0.8, 0.8, 0.8]);
hold on;

% Create complete paths (initially hidden)
full_x_path = plot3(x_axis_history(:,1), x_axis_history(:,2), x_axis_history(:,3), 'r-', 'LineWidth', 1, 'Color', [1 0 0 0.3]);
full_y_path = plot3(y_axis_history(:,1), y_axis_history(:,2), y_axis_history(:,3), 'g-', 'LineWidth', 1, 'Color', [0 1 0 0.3]);
full_z_path = plot3(z_axis_history(:,1), z_axis_history(:,2), z_axis_history(:,3), 'b-', 'LineWidth', 1, 'Color', [0 0 1 0.3]);

% Create paths that will grow with slider (initially only first point)
x_path_h = plot3(x_axis_history(1,1), x_axis_history(1,2), x_axis_history(1,3), 'r-', 'LineWidth', 2);
y_path_h = plot3(y_axis_history(1,1), y_axis_history(1,2), y_axis_history(1,3), 'g-', 'LineWidth', 2);
z_path_h = plot3(z_axis_history(1,1), z_axis_history(1,2), z_axis_history(1,3), 'b-', 'LineWidth', 2);

% Plot angular momentum direction (fixed in space)
quiver3(0, 0, 0, L_dir(1), L_dir(2), L_dir(3), 'k-', 'LineWidth', 3, 'MaxHeadSize', 0.5);
text(L_dir(1)*1.1, L_dir(2)*1.1, L_dir(3)*1.1, 'L', 'FontSize', 14);

% Create markers for current positions
x_marker = plot3(x_axis_history(1,1), x_axis_history(1,2), x_axis_history(1,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
y_marker = plot3(y_axis_history(1,1), y_axis_history(1,2), y_axis_history(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
z_marker = plot3(z_axis_history(1,1), z_axis_history(1,2), z_axis_history(1,3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Add a smaller version of the cube within the sphere
% Create unit cube vertices but scaled down to fit within sphere
scale_factor = 0.6; % Increased from 0.4 to make cube bigger
small_vertices = vertices * scale_factor;

% Apply initial rotation to match orientation at t=0
q_init = state(1,1:4);
dcm_init = quat2dcm(q_init);
rotated_small_vertices = (dcm_init * small_vertices')';

% Create cube patch in sphere plot with less transparency
small_cube_h = patch('Vertices', rotated_small_vertices, 'Faces', faces, ...
                    'FaceColor', 'flat', 'FaceVertexCData', face_colors, ...
                    'EdgeColor', 'k', 'LineWidth', 1);
alpha(small_cube_h, 0.5);  % Less transparent (was 0.3)

% Add principal axes to the cube in sphere plot
small_axes_h = zeros(3,1);
small_axis_length = scale_factor * axis_length;  % Scale to match cube size

for i = 1:3
    % Create line for each principal axis
    xyz = zeros(2,3);
    xyz(2,i) = small_axis_length;  % Axis points along coordinate axis
    
    % Apply initial orientation
    xyz_rot = (dcm_init * xyz')';
    
    % Create the axis line
    small_axes_h(i) = line('XData', xyz_rot(:,1), 'YData', xyz_rot(:,2), 'ZData', xyz_rot(:,3), ...
                         'Color', axes_colors{i}, 'LineWidth', 2);
end

% Store handles to the small axes for updating in the slider callback
sliderData.small_axes_h = small_axes_h;
sliderData.small_axis_length = small_axis_length;

% Apply these fixes to the slider data structure to include the cube
sliderData.small_cube_h = small_cube_h;
sliderData.small_vertices = small_vertices;

% Set plot properties
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Motion of Principal Axes on Unit Sphere', 'FontSize', 14);
legend('Unit sphere', 'X-axis full path', 'Y-axis full path', 'Z-axis path', ...
       'X-axis current path', 'Y-axis current path', 'Z-axis current path', ...
       'Angular momentum', 'X-axis position', 'Y-axis position', 'Z-axis position', ...
       'Rotating cube', 'Location', 'eastoutside');

% Add view control
view(30, 20);

% Add time text display
time_text = text(-1.5, -1.5, 1.5, ['Time: ' num2str(t(1)) ' s'], 'FontSize', 12);

% Add quaternion text display
quat_text = text(-1.5, -1.5, 1.3, ['q = [' num2str(state(1,1:4), '%.3f, ') ']'], 'FontSize', 12);

% Create a data structure to store all the handles needed for the slider callback
sliderData.x_marker = x_marker;
sliderData.y_marker = y_marker;
sliderData.z_marker = z_marker;
sliderData.x_path_h = x_path_h;
sliderData.y_path_h = y_path_h;
sliderData.z_path_h = z_path_h;
sliderData.time_text = time_text;
sliderData.quat_text = quat_text;  % Add quaternion text to sliderData
sliderData.x_axis_history = x_axis_history;
sliderData.y_axis_history = y_axis_history;
sliderData.z_axis_history = z_axis_history;
sliderData.t = t;
sliderData.sphere_fig = sphere_fig;
sliderData.full_x_path = full_x_path;
sliderData.full_y_path = full_y_path;
sliderData.full_z_path = full_z_path;
sliderData.show_all = true;
sliderData.show_x = true;
sliderData.show_y = true;
sliderData.show_z = true;
sliderData.state = state;  % Store the state variable in slider data

% Create slider panel
slider_panel = uipanel('Parent', sphere_fig, 'Position', [0.1, 0.02, 0.8, 0.06]);
time_slider = uicontrol('Parent', slider_panel, ...
                       'Style', 'slider', ...
                       'Min', 1, 'Max', n_steps, 'Value', 1, ...
                       'Units', 'normalized', ...
                       'Position', [0.15, 0.2, 0.7, 0.6], ...
                       'Tag', 'timeSlider', ...
                       'UserData', sliderData);
                   
% Set up callback for the slider
set(time_slider, 'Callback', @(src, event) updateMarkers(src, event, sliderData));

% Create play button
play_button = uicontrol('Parent', slider_panel, ...
                         'Style', 'pushbutton', ...
                         'String', 'Play', ...
                         'Units', 'normalized', ...
                         'Position', [0.02, 0.2, 0.1, 0.6], ...
                         'Tag', 'playButton');

% Create a new structure for the play button's userData
playData = struct('timer', [], 'is_playing', false, 'slider', time_slider);
                         
% Set up callback for the play button
set(play_button, 'UserData', playData);
set(play_button, 'Callback', @togglePlayback);

% Create axis isolation buttons
button_panel = uipanel('Parent', sphere_fig, 'Position', [0.05, 0.92, 0.9, 0.06]);

% All axes button
all_button = uicontrol('Parent', button_panel, ...
                      'Style', 'pushbutton', ...
                      'String', 'Show All', ...
                      'Units', 'normalized', ...
                      'Position', [0.05, 0.1, 0.2, 0.8], ...
                      'Tag', 'allButton', ...
                      'UserData', sliderData);
set(all_button, 'Callback', @(src, event) showAll(src, event, time_slider));

% X axis button
x_button = uicontrol('Parent', button_panel, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show X Only', ...
                     'Units', 'normalized', ...
                     'Position', [0.3, 0.1, 0.2, 0.8], ...
                     'BackgroundColor', [1, 0.8, 0.8], ...
                     'Tag', 'xButton', ...
                     'UserData', sliderData);
set(x_button, 'Callback', @(src, event) showXOnly(src, event, time_slider));

% Y axis button
y_button = uicontrol('Parent', button_panel, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show Y Only', ...
                     'Units', 'normalized', ...
                     'Position', [0.55, 0.1, 0.2, 0.8], ...
                     'BackgroundColor', [0.8, 1, 0.8], ...
                     'Tag', 'yButton', ...
                     'UserData', sliderData);
set(y_button, 'Callback', @(src, event) showYOnly(src, event, time_slider));

% Z axis button
z_button = uicontrol('Parent', button_panel, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show Z Only', ...
                     'Units', 'normalized', ...
                     'Position', [0.8, 0.1, 0.2, 0.8], ...
                     'BackgroundColor', [0.8, 0.8, 1], ...
                     'Tag', 'zButton', ...
                     'UserData', sliderData);
set(z_button, 'Callback', @(src, event) showZOnly(src, event, time_slider));

% Save handle to the figure for later reference
sphere_slider_fig = sphere_fig;

% Define the marker update function separately so it's in the base workspace
function updateMarkers(hObject, ~, ~)
    % Get the current time index from the slider
    idx = round(get(hObject, 'Value'));
    
    % Get the latest data in case it has been modified by other functions
    data = get(hObject, 'UserData');
    
    % Update marker positions
    set(data.x_marker, 'XData', data.x_axis_history(idx,1), ...
                      'YData', data.x_axis_history(idx,2), ...
                      'ZData', data.x_axis_history(idx,3), ...
                      'Visible', iif(data.show_x, 'on', 'off'));
                  
    set(data.y_marker, 'XData', data.y_axis_history(idx,1), ...
                      'YData', data.y_axis_history(idx,2), ...
                      'ZData', data.y_axis_history(idx,3), ...
                      'Visible', iif(data.show_y, 'on', 'off'));
                  
    set(data.z_marker, 'XData', data.z_axis_history(idx,1), ...
                      'YData', data.z_axis_history(idx,2), ...
                      'ZData', data.z_axis_history(idx,3), ...
                      'Visible', iif(data.show_z, 'on', 'off'));
    
    % Update paths up to current point
    set(data.x_path_h, 'XData', data.x_axis_history(1:idx,1), ...
                      'YData', data.x_axis_history(1:idx,2), ...
                      'ZData', data.x_axis_history(1:idx,3), ...
                      'Visible', iif(data.show_x, 'on', 'off'));
                  
    set(data.y_path_h, 'XData', data.y_axis_history(1:idx,1), ...
                      'YData', data.y_axis_history(1:idx,2), ...
                      'ZData', data.y_axis_history(1:idx,3), ...
                      'Visible', iif(data.show_y, 'on', 'off'));
                  
    set(data.z_path_h, 'XData', data.z_axis_history(1:idx,1), ...
                      'YData', data.z_axis_history(1:idx,2), ...
                      'ZData', data.z_axis_history(1:idx,3), ...
                      'Visible', iif(data.show_z, 'on', 'off'));
    
    % Get the quaternion for the current time from stored state
    q = data.state(idx,1:4);
    dcm = quat2dcm(q);
    
    % Update the cube orientation
    if isfield(data, 'small_cube_h') && ishandle(data.small_cube_h)
        rotated_small_vertices = (dcm * data.small_vertices')';
        set(data.small_cube_h, 'Vertices', rotated_small_vertices);
    end
    
    % Update the small axes on the cube
    if isfield(data, 'small_axes_h')
        for j = 1:3
            if ishandle(data.small_axes_h(j))
                xyz = zeros(2,3);
                xyz(2,j) = data.small_axis_length;  % Axis points along coordinate axis
                
                % Apply current orientation
                xyz_rot = (dcm * xyz')';
                
                % Update the axis line
                set(data.small_axes_h(j), 'XData', xyz_rot(:,1), 'YData', xyz_rot(:,2), 'ZData', xyz_rot(:,3));
            end
        end
    end
    
    % Update time text
    if isfield(data, 'time_text') && ishandle(data.time_text)
        set(data.time_text, 'String', ['Time: ' num2str(data.t(idx), '%.2f') ' s']);
    end
    
    % Update quaternion text
    if isfield(data, 'quat_text') && ishandle(data.quat_text)
        q_str = sprintf('q = [%.3f, %.3f, %.3f, %.3f]', data.state(idx, 1:4));
        set(data.quat_text, 'String', q_str);
    end
    
    % Force the figure to update now
    if isfield(data, 'sphere_fig') && ishandle(data.sphere_fig)
        figure(data.sphere_fig);
        drawnow;
    end
end

% Replace the togglePlayback function with this version that includes an option to stop at end
function togglePlayback(hObject, ~)
    % Get user data with playback state and objects
    userData = get(hObject, 'UserData');
    
    if ~userData.is_playing
        % START ANIMATION
        userData.is_playing = true;
        set(hObject, 'String', 'Pause');
        
        % Delete any existing timers
        if ~isempty(userData.timer) && isvalid(userData.timer)
            stop(userData.timer);
            delete(userData.timer);
        end
        
        % Create a persistent timer for playback (without infinite tasks)
        userData.timer = timer('ExecutionMode', 'FixedRate', ...
                              'Period', 0.05, ... % 20fps
                              'TimerFcn', {@animationTimerCallback, userData.slider, hObject});
        
        set(hObject, 'UserData', userData);
        start(userData.timer);
    else
        % STOP ANIMATION
        userData.is_playing = false;
        set(hObject, 'String', 'Play');
        
        % Stop and delete the timer
        if ~isempty(userData.timer) && isvalid(userData.timer)
            stop(userData.timer);
            delete(userData.timer);
            userData.timer = [];
        end
        
        set(hObject, 'UserData', userData);
    end
end

% Create a new animation timer callback function
function animationTimerCallback(~, ~, slider, playButton)
    try
        % Get the button data to check if we're still playing
        buttonData = get(playButton, 'UserData');
        if ~buttonData.is_playing
            return;
        end
        
        % Get slider data
        sliderData = get(slider, 'UserData');
        
        % Get current position
        currentPos = round(get(slider, 'Value'));
        
        % Check array bounds to avoid errors
        maxPos = length(sliderData.t);
        
        if currentPos >= maxPos
            % We've reached the end - reset to beginning and stop playback
            set(slider, 'Value', 1);
            updateMarkers(slider, [], sliderData);
            
            % Stop playback
            buttonData.is_playing = false;
            set(playButton, 'String', 'Play');
            
            % Stop and delete the timer
            if ~isempty(buttonData.timer) && isvalid(buttonData.timer)
                stop(buttonData.timer);
                delete(buttonData.timer);
                buttonData.timer = [];
            end
            
            % Update button state
            set(playButton, 'UserData', buttonData);
            
            return;
        else
            % Normal playback - advance by 2 frames but check max bound
            newPos = min(currentPos + 2, maxPos);
            set(slider, 'Value', newPos);
        end
        
        % Call the slider's callback function directly
        updateMarkers(slider, [], sliderData);
        
    catch ME
        disp(['Animation error: ' ME.message]);
    end
end



%% Plotting components of the principal axes
% X components of all principal axes
figure('Name', 'X Components of Principal Axes', 'Position', [100, 100, 900, 600]);

% X component of X-axis
subplot(3,1,1);
plot(t, x_axis_history(:,1), 'r-', 'LineWidth', 2); hold on;
yline(L_dir(1), 'k--', 'L_x', 'LineWidth', 1.5);
title('X Component of X Principal Axis', 'FontSize', 12);
grid on;
ylabel('Value');

% X component of Y-axis
subplot(3,1,2);
plot(t, y_axis_history(:,1), 'g-', 'LineWidth', 2); hold on;
yline(L_dir(1), 'k--', 'L_x', 'LineWidth', 1.5);
title('X Component of Y Principal Axis', 'FontSize', 12);
grid on;
ylabel('Value');

% X component of Z-axis
subplot(3,1,3);
plot(t, z_axis_history(:,1), 'b-', 'LineWidth', 2); hold on;
yline(L_dir(1), 'k--', 'L_x', 'LineWidth', 1.5);
title('X Component of Z Principal Axis', 'FontSize', 12);
grid on;
xlabel('Time (s)');
ylabel('Value');

% Add a common title
sgtitle('X Components of All Principal Axes', 'FontSize', 14);

% Y components of all principal axes
figure('Name', 'Y Components of Principal Axes', 'Position', [150, 150, 900, 600]);

% Y component of X-axis
subplot(3,1,1);
plot(t, x_axis_history(:,2), 'r-', 'LineWidth', 2); hold on;
yline(L_dir(2), 'k--', 'L_y', 'LineWidth', 1.5);
title('Y Component of X Principal Axis', 'FontSize', 12);
grid on;
ylabel('Value');

% Y component of Y-axis
subplot(3,1,2);
plot(t, y_axis_history(:,2), 'g-', 'LineWidth', 2); hold on;
yline(L_dir(2), 'k--', 'L_y', 'LineWidth', 1.5);
title('Y Component of Y Principal Axis', 'FontSize', 12);
grid on;
ylabel('Value');

% Y component of Z-axis
subplot(3,1,3);
plot(t, z_axis_history(:,2), 'b-', 'LineWidth', 2); hold on;
yline(L_dir(2), 'k--', 'L_y', 'LineWidth', 1.5);
title('Y Component of Z Principal Axis', 'FontSize', 12);
grid on;
xlabel('Time (s)');
ylabel('Value');

% Add a common title
sgtitle('Y Components of All Principal Axes', 'FontSize', 14);

% Z components of all principal axes
figure('Name', 'Z Components of Principal Axes', 'Position', [200, 200, 900, 600]);

% Z component of X-axis
subplot(3,1,1);
plot(t, x_axis_history(:,3), 'r-', 'LineWidth', 2); hold on;
yline(L_dir(3), 'k--', 'L_z', 'LineWidth', 1.5);
title('Z Component of X Principal Axis', 'FontSize', 12);
grid on;
ylabel('Value');

% Z component of Y-axis
subplot(3,1,2);
plot(t, y_axis_history(:,3), 'g-', 'LineWidth', 2); hold on;
yline(L_dir(3), 'k--', 'L_z', 'LineWidth', 1.5);
title('Z Component of Y Principal Axis', 'FontSize', 12);
grid on;
ylabel('Value');

% Z component of Z-axis
subplot(3,1,3);
plot(t, z_axis_history(:,3), 'b-', 'LineWidth', 2); hold on;
yline(L_dir(3), 'k--', 'L_z', 'LineWidth', 1.5);
title('Z Component of Z Principal Axis', 'FontSize', 12);
grid on;
xlabel('Time (s)');
ylabel('Value');

% Add a common title
sgtitle('Z Components of All Principal Axes', 'FontSize', 14);

% Add this helper function at the end of your file
function result = iif(condition, trueVal, falseVal)
    % Simple inline if implementation
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end

% Function to show all axes
function showAll(~, ~, slider)
    % Get slider data
    sliderData = get(slider, 'UserData');
    
    % Show all axes
    sliderData.show_x = true;
    sliderData.show_y = true;
    sliderData.show_z = true;
    sliderData.show_all = true;
    
    % Update the slider's UserData
    set(slider, 'UserData', sliderData);
    
    % Update the display
    updateMarkers(slider, [], sliderData);
end

% Function to show only X axis
function showXOnly(~, ~, slider)
    % Get slider data
    sliderData = get(slider, 'UserData');
    
    % Show only X axis
    sliderData.show_x = true;
    sliderData.show_y = false;
    sliderData.show_z = false;
    sliderData.show_all = false;
    
    % Update the slider's UserData
    set(slider, 'UserData', sliderData);
    
    % Update the display
    updateMarkers(slider, [], sliderData);
end

% Function to show only Y axis
function showYOnly(~, ~, slider)
    % Get slider data
    sliderData = get(slider, 'UserData');
    
    % Show only Y axis
    sliderData.show_x = false;
    sliderData.show_y = true;
    sliderData.show_z = false;
    sliderData.show_all = false;
    
    % Update the slider's UserData
    set(slider, 'UserData', sliderData);
    
    % Update the display
    updateMarkers(slider, [], sliderData);
end

% Function to show only Z axis
function showZOnly(~, ~, slider)
    % Get slider data
    sliderData = get(slider, 'UserData');
    
    % Show only Z axis
    sliderData.show_x = false;
    sliderData.show_y = false;
    sliderData.show_z = true;
    sliderData.show_all = false;
    
    % Update the slider's UserData
    set(slider, 'UserData', sliderData);
    
    % Update the display
    updateMarkers(slider, [], sliderData);
end

% Add this code after creating the sphere_fig (around line 310):

% Add a DeleteFcn to clean up timers when figure closes
set(sphere_fig, 'DeleteFcn', @cleanupTimers);

% And add this helper function at the end of your file:
function cleanupTimers(~, ~)
    % Find all active timers
    t = timerfindall;
    
    % Delete any existing timers
    if ~isempty(t)
        stop(t);
        delete(t);
    end
    
    % Clear any persistent variables
    
end






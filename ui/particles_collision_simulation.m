function varargout = particles_collision_simulation(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @particles_collision_simulation_OpeningFcn, ...
    'gui_OutputFcn',  @particles_collision_simulation_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT



% --- Executes just before particles_collision_simulation is made visible.
function particles_collision_simulation_OpeningFcn(hObject, eventdata, handles, varargin)

handles.X_BOUND = 10;
handles.Y_BOUND = 10;
handles.max_particle_size = 0.3;
handles.particle_count = 100;

handles.output = hObject;
handles.last_clicked_particle=0;
handles.particle_radius_vector = abs(normrnd(0.1,0.1,[1 handles.particle_count]));
handles.particle_mass_vector = ones(1,handles.particle_count);
handles.color_map = hsv(handles.particle_count);
[handles.x, handles.y] = do_initialize_box(handles);
[handles.vx, handles.vy] = do_initialize_velocity_vectors(handles.particle_count);
guidata(hObject, handles);
set(handles.reset_button,'Enable','off');
handles.particles_plot = do_plot_particles(handles);
handles.drag_line = arrow('Start',[0 0 0],'Stop',[0 0 0],'BaseAngle',80, 'Length',0);
set(handles.reset_button,'Enable','on');
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = particles_collision_simulation_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;


% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
try
    stop(handles.timer);
catch
end
handles.particle_count = floor(get(handles.particle_count_sld,'Value'));
[handles.particle_radius_vector, handles.particle_mass_vector] = do_init_configuration_inputs(handles);
handles.color_map = hsv(handles.particle_count);
[handles.x,handles.y] = do_initialize_box(handles);
[handles.vx, handles.vy] = do_initialize_velocity_vectors(handles.particle_count);
handles.drag_line = arrow('Start',[0 0 0],'Stop',[0 0 0],'BaseAngle',80, 'Length',0);
handles.particles_plot = do_plot_particles(handles);
guidata(hObject, handles);



% --- Executes on button press in stop_button.
function stop_button_Callback(hObject, eventdata, handles)
try
    stop(handles.timer);
catch ME
    disp(ME);
end
set_start_stop_enabled(handles,'on','off');
guidata(hObject, handles);

% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
try
    start(handles.timer);
    set_start_stop_enabled(handles,'off','on');
    guidata(hObject, handles);
catch ME
    disp(ME);
end

function set_start_stop_enabled(handles, start_val, stop_val)
set(handles.stop_button,'Enable',stop_val);
set(handles.start_button,'Enable',start_val);

% --- Executes on slider movement.
function particle_count_sld_Callback(hObject, eventdata, handles)
val = floor(get(hObject,'Value'));
set(handles.particle_count_text_box,'String',num2str(val));
set(handles.text3, 'String', resolve_label_text(val,handles));
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function particle_count_sld_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function particle_count_text_box_Callback(hObject, eventdata, handles)
set(handles.particle_count_sld,'Value',str2double(get(hObject,'String')));

% --- Executes during object creation, after setting all properties.
function particle_count_text_box_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function particle_radius_text_box_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function particle_radius_text_box_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radio_value_radius.
function radio_value_radius_Callback(hObject, eventdata, handles)
% hObject    handle to radio_value_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value_radio_callback(hObject,handles.radio_random_radius,handles.particle_radius_text_box,'on','off');
guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of radio_value_radius



% --- Executes on button press in radio_random_radius.
function radio_random_radius_Callback(hObject, eventdata, handles)

value_radio_callback(hObject,handles.radio_value_radius,handles.particle_radius_text_box,'off','on');
guidata(hObject,handles);


% --- Executes on button press in particle_mass_value_button.
function particle_mass_value_button_Callback(hObject, eventdata, handles)

value_radio_callback(hObject,handles.particle_random_mass_button,handles.particle_mass_text_box,'on','off');
guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of particle_mass_value_button



function particle_mass_text_box_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function particle_mass_text_box_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in particle_random_mass_button.
function particle_random_mass_button_Callback(hObject, eventdata, handles)
% hObject    handle to particle_random_mass_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value_radio_callback(hObject,handles.particle_mass_value_button,handles.particle_mass_text_box,'off','on');
guidata(hObject,handles);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    stop(handles.timer);
catch
end
axes(handles.particle_box);
cla;
axes(handles.velocity_distribution);
cla;
delete(hObject);

% --- Executes on mouse press over axes background.
function particle_box_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to particle_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clicked_point = get(handles.particle_box,'CurrentPoint');
xp1 = clicked_point(1);
yp1 = clicked_point(3);
particle_distance = calculate_particles_distance(xp1,yp1,handles.x,handles.y);
for i=1:1:handles.particle_count
    particleID = find(particle_distance < handles.particle_radius_vector(i));
end
handles.particleID = particleID;
guidata(hObject, handles);


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
current_point = get(handles.particle_box,'CurrentPoint');
if handles.last_clicked_particle
    arrow(handles.drag_line,'Start',[0 0 0],'Stop',[0 0 0],'BaseAngle',80, 'Length',0);
    handles.vx(handles.last_clicked_particle) = current_point(1) - handles.x(handles.last_clicked_particle);
    handles.vy(handles.last_clicked_particle) = current_point(3) - handles.y(handles.last_clicked_particle);
    guidata(hObject, handles);
    set(handles.stop_button,'Enable','on');
    handles.timer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.01, ...
        'TimerFcn', @(x,y)(timerFcn(x,y,handles.figure1)), 'StartDelay', 0.2, 'TasksToExecute', inf );
    start(handles.timer);
end
handles.last_clicked_particle = 0;
guidata(hObject, handles);


% --- Executes on mouse motion over figure - except title and menu.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
if handles.last_clicked_particle ~= 0
    i = handles.last_clicked_particle;
    current_point = get(handles.particle_box,'CurrentPoint');
    if is_in_plot_area(handles, current_point, handles.particle_radius_vector(i))
        arrow(handles.drag_line,'Start',[handles.x(i) handles.y(i) 0],'Stop',[current_point(1) current_point(3) 0],'BaseAngle',80, 'Length',10);
    end
end
guidata(hObject, handles);

function [b] = is_in_plot_area(handles, current_point, particle_radius)
b = 0;
if current_point(1)-particle_radius > 0 && current_point(1) + particle_radius < handles.X_BOUND && current_point(3)-particle_radius > 0 && current_point(3)+particle_radius < handles.Y_BOUND
    b = 1;
end


function [circles] = do_plot_particles(handles)
circles = zeros(1,handles.particle_count);
set_buttons_enabled(handles,'off');
for i=1:1:handles.particle_count
    circles(i) = do_plot_single_particle(handles.x(i),handles.y(i),i, handles.particle_radius_vector(i), handles.color_map(i,:),handles);
    pause(0.001);
end
set(handles.reset_button,'Enable','on');

function set_buttons_enabled(handles,value)
set(handles.reset_button,'Enable',value);
set(handles.start_button,'Enable',value);
set(handles.stop_button,'Enable',value);


function [circle] =  do_plot_single_particle(x,y,particle_ID, radius, color_map, handles)
circle = rectangle('FaceColor',color_map,...
    'Curvature',[1,1],...
    'Position',[x-radius,y-radius,...
    2*radius,2*radius],...
    'ButtonDownFcn',{@particle_click_callback,handles,particle_ID});

function update_distribution_plot(handles, vx, vy)
v = sqrt(vx.^2 + vy.^2);
ylim(handles.velocity_distribution,[0 1]);
histogram(handles.velocity_distribution,v,10,'Normalization','probability');
ylim(handles.velocity_distribution,[0 1]);

function timerFcn(obj, event, gui)
try
    handles = guidata(gui);
    [handles.x, handles.y, handles.vx, handles.vy] = do_particle_movement_simulation(handles.x, handles.y, handles.vx, handles.vy, handles.particle_radius_vector, handles.particle_mass_vector, handles.X_BOUND, handles.Y_BOUND);
    for i=1:1:handles.particle_count
        set(handles.particles_plot(i), 'Position', [handles.x(i)-handles.particle_radius_vector(i),handles.y(i)-handles.particle_radius_vector(i),...
            2*handles.particle_radius_vector(i),2*handles.particle_radius_vector(i)]);
    end
    axes(handles.velocity_distribution);
    update_distribution_plot(handles,handles.vx, handles.vy);
    guidata(gui, handles);
catch ME
    disp (ME)
end

function value_radio_callback(current_object, target_radio, target_input, on_active_radio_action, on_deactive_radio_action)
if get(current_object,'Value')==0
    set(target_input,'Visible',on_deactive_radio_action);
    set(target_radio,'Value',1);
else
    set(target_input,'Visible',on_active_radio_action);
    set(target_radio,'Value',0);
end

function [label_text] = resolve_label_text(val, handles)
if val > 100
    handles.max_particle_size = 0.2;
    label_text = 'Choose particle radius (0.001 - 0.2)';
elseif val > 50
    handles.max_particle_size = 0.3;
    label_text = 'Choose particle radius (0.001 - 0.3)';
else
    handles.max_particle_size = 0.5;
    label_text = 'Choose particle radius (0.001 - 0.5)';
end

function [x,y] = do_initialize_box(handles)
reset_axes(handles);
axis(handles.particle_box,[0 handles.X_BOUND 0 handles.Y_BOUND]);
set(handles.reset_button,'enable','off');
axes(handles.particle_box);
[x,y] = initialize_particle_box(handles.particle_count, handles.particle_radius_vector, handles.particle_mass_vector, handles.X_BOUND, handles.Y_BOUND, handles.color_map);
set(handles.reset_button,'enable','on');


function reset_axes(handles)
cla(handles.particle_box);
cla(handles.velocity_distribution);

function [vx, vy] = do_initialize_velocity_vectors(particle_count)
vx = zeros(1,particle_count);
vy = zeros(1,particle_count);


function [particle_radius_vector, particle_mass_vector] = do_init_configuration_inputs(handles)
random_data = abs(normrnd(0.1,0.1,[1 handles.particle_count]));
if get(handles.radio_value_radius,'Value') == 1
    if(handles.max_particle_size < str2double(get(handles.particle_radius_text_box, 'String')))
        errordlg('Selected particle size is too big, please change particle size value');
        return;
    end
    particle_radius_vector = ones(1,handles.particle_count).*str2double(get(handles.particle_radius_text_box, 'String'));
else
    particle_radius_vector = random_data;
end
if get(handles.particle_mass_value_button,'Value') == 1
    particle_mass_vector = ones(1,handles.particle_count).*str2double(get(handles.particle_mass_text_box, 'String'));
else
    particle_mass_vector = random_data;
end


function particle_click_callback(src,evt, handles, particle_ID)
myhandles = guidata(gcbo);
myhandles.last_clicked_particle = particle_ID;
guidata(gcbo,myhandles)

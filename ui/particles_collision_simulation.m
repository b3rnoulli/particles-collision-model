function varargout = particles_collision_simulation(varargin)
% PARTICLES_COLLISION_SIMULATION MATLAB code for particles_collision_simulation.fig
%      PARTICLES_COLLISION_SIMULATION, by itself, creates a new PARTICLES_COLLISION_SIMULATION or raises the existing
%      singleton*.
%
%      H = PARTICLES_COLLISION_SIMULATION returns the handle to a new PARTICLES_COLLISION_SIMULATION or the handle to
%      the existing singleton*.
%
%      PARTICLES_COLLISION_SIMULATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PARTICLES_COLLISION_SIMULATION.M with the given input arguments.
%
%      PARTICLES_COLLISION_SIMULATION('Property','Value',...) creates a new PARTICLES_COLLISION_SIMULATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before particles_collision_simulation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to particles_collision_simulation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help particles_collision_simulation

% Last Modified by GUIDE v2.5 24-Jan-2015 00:04:33

% Begin initialization code - DO NOT EDIT
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
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to particles_collision_simulation (see VARARGIN)

% Choose default command line output for particles_collision_simulation
handles.X_BOUND = 10;
handles.Y_BOUND = 10;
handles.output = hObject;
handles.last_clicked_particle=0;
handles.particle_count = 100;
handles.particle_radius_vector = abs(normrnd(0.1,0.1,[1 handles.particle_count]));
handles.particle_mass_vector = ones(1,handles.particle_count);
handles.color_map = hsv(handles.particle_count);
[handles.x, handles.y] = do_initialize_box(handles);
guidata(hObject, handles);
set(handles.reset_button,'Enable','off');
handles.particles_plot = do_plot_particles(handles);
set(handles.reset_button,'Enable','on');
guidata(hObject, handles);

function particle_click_callback(src,evt, handles, particle_ID)
myhandles = guidata(gcbo);
myhandles.last_clicked_particle = particle_ID;
guidata(gcbo,myhandles)


% UIWAIT makes particles_collision_simulation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = particles_collision_simulation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
% hObject    handle to reset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.particle_count = floor(get(handles.particle_count_sld,'Value'));
random_data = abs(normrnd(0.1,0.1,[1 handles.particle_count]));
if get(handles.radio_value_radius,'Value') == 1
     handles.particle_radius_vector = ones(1,handles.particle_count).*str2double(get(handles.particle_radius_text_box, 'String'));
else
    handles.particle_radius_vector = random_data;
end
if get(handles.particle_mass_value_button,'Value') == 1
    handles.particle_mass_vector = ones(1,handles.particle_count).*str2double(get(handles.particle_mass_text_box, 'String'));
else
    handles.particle_mass_vector= random_data;
end
handles.color_map = hsv(handles.particle_count);
axes(handles.particle_box);
[handles.x,handles.y] = do_initialize_box(handles);
guidata(hObject, handles);
set(handles.reset_button,'Enable','off');
do_plot_particles(handles);
set(handles.reset_button,'Enable','on');



% --- Executes on button press in stop_button.
function stop_button_Callback(hObject, eventdata, handles)
% hObject    handle to stop_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function particle_count_sld_Callback(hObject, eventdata, handles)
% hObject    handle to particle_count_sld (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
val = floor(get(hObject,'Value'));
set(handles.particle_count_text_box,'String',num2str(val));
set(handles.text3, 'String', resolve_label_text(val));
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function particle_count_sld_CreateFcn(hObject, eventdata, handles)
% hObject    handle to particle_count_sld (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function particle_count_text_box_Callback(hObject, eventdata, handles)
% hObject    handle to particle_count_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.particle_count_sld,'Value',str2double(get(hObject,'String')));



% --- Executes during object creation, after setting all properties.
function particle_count_text_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to particle_count_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function particle_radius_text_box_Callback(hObject, eventdata, handles)
% hObject    handle to particle_radius_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function particle_radius_text_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to particle_radius_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
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


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in radio_random_radius.
function radio_random_radius_Callback(hObject, eventdata, handles)
% hObject    handle to radio_random_radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value_radio_callback(hObject,handles.radio_value_radius,handles.particle_radius_text_box,'off','on');
guidata(hObject,handles);


% --- Executes on button press in particle_mass_value_button.
function particle_mass_value_button_Callback(hObject, eventdata, handles)
% hObject    handle to particle_mass_value_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value_radio_callback(hObject,handles.particle_random_mass_button,handles.particle_mass_text_box,'on','off');
guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of particle_mass_value_button



function particle_mass_text_box_Callback(hObject, eventdata, handles)
% hObject    handle to particle_mass_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of particle_mass_text_box as text
%        str2double(get(hObject,'String')) returns contents of particle_mass_text_box as a double


% --- Executes during object creation, after setting all properties.
function particle_mass_text_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to particle_mass_text_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
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
% Hint: get(hObject,'Value') returns toggle state of particle_random_mass_button


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

axes(handles.particle_box);
cla;
axes(handles.velocity_distribution);
delete(hObject);

function value_radio_callback(current_object, target_radio, target_input, on_active_radio_action, on_deactive_radio_action)
if get(current_object,'Value')==0
    set(target_input,'Visible',on_deactive_radio_action);
    set(target_radio,'Value',1);
else
    set(target_input,'Visible',on_active_radio_action);
    set(target_radio,'Value',0);
end

function [label_text] = resolve_label_text(val)
if val > 100
    label_text = 'Choose particle radius (0.001 - 0.2)';
elseif val > 50
    label_text = 'Choose particle radius (0.001 - 0.3)';
else
    label_text = 'Choose particle radius (0.001 - 0.5)';
end


function [x,y] = do_initialize_box(handles)
axes(handles.particle_box);
cla;
axis([0 handles.X_BOUND 0 handles.Y_BOUND]);
set(handles.reset_button,'enable','off');
[x,y] = initialize_particle_box(handles.particle_count, handles.particle_radius_vector, handles.particle_mass_vector, handles.X_BOUND, handles.Y_BOUND, handles.color_map);
set(handles.reset_button,'enable','on');


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
handles.last_clicked_particle = 0;
guidata(hObject, handles);


% --- Executes on mouse motion over figure - except title and menu.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.last_clicked_particle ~= 0
   i= handles.last_clicked_particle;
   current_point = get(handles.particle_box,'CurrentPoint');
   delete(handles.particles_plot(i));
   handles.particles_plot(i) = do_plot_single_particle(current_point(1),current_point(3),i,handles.particle_radius_vector(i),handles.color_map(i,:), handles);  
end
guidata(hObject, handles);

function [circles] = do_plot_particles(handles)
circles = zeros(1,handles.particle_count);
for i=1:1:handles.particle_count
    circles(i) = do_plot_single_particle(handles.x(i),handles.y(i),i, handles.particle_radius_vector(i), handles.color_map(i,:),handles);
    pause(0.001); 
end

function [circle] =  do_plot_single_particle(x,y,particle_ID, radius, color_map, handles)
circle = rectangle('FaceColor',color_map,...
        'Curvature',[1,1],...
        'Position',[x-radius,y-radius,...
        2*radius,2*radius],...
        'ButtonDownFcn',{@particle_click_callback,handles,particle_ID});




function varargout = RR_robot_sim(varargin)
% RR_robot_sim MATLAB code for RR_robot_sim.fig
%      RR_robot_sim, by itself, creates a new RR_robot_sim or raises the existing
%      singleton*.
%
%      H = RR_robot_sim returns the handle to a new RR_robot_sim or the handle to
%      the existing singleton*.
%
%      RR_robot_sim('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RR_robot_sim.M with the given input arguments.
%
%      RR_robot_sim('Property','Value',...) creates a new RR_robot_sim or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RR_robot_sim_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RR_robot_sim_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RR_robot_sim

% Last Modified by GUIDE v2.5 15-Aug-2025 16:20:52

% run initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RR_robot_sim_OpeningFcn, ...
                   'gui_OutputFcn',  @RR_robot_sim_OutputFcn, ...
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


% --- Executes just before RR_robot_sim is made visible.
function RR_robot_sim_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RR_robot_sim (see VARARGIN)

% Choose default command line output for RR_robot_sim
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RR_robot_sim wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RR_robot_sim_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function x_i_Callback(hObject, eventdata, handles)
% hObject    handle to x_i (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_i as text
%        str2double(get(hObject,'String')) returns contents of x_i as a double


% --- Executes during object creation, after setting all properties.
function x_i_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_i (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function x_f_Callback(hObject, eventdata, handles)
% hObject    handle to x_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_f as text
%        str2double(get(hObject,'String')) returns contents of x_f as a double


% --- Executes during object creation, after setting all properties.
function x_f_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function y_i_Callback(hObject, eventdata, handles)
% hObject    handle to y_i (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_i as text
%        str2double(get(hObject,'String')) returns contents of y_i as a double


% --- Executes during object creation, after setting all properties.
function y_i_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_i (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function y_f_Callback(hObject, eventdata, handles)
% hObject    handle to y_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_f as text
%        str2double(get(hObject,'String')) returns contents of y_f as a double


% --- Executes during object creation, after setting all properties.
function y_f_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function a_1_Callback(hObject, eventdata, handles)
% hObject    handle to a_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of a_1 as text
%        str2double(get(hObject,'String')) returns contents of a_1 as a double


% --- Executes during object creation, after setting all properties.
function a_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to a_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function a_2_Callback(hObject, eventdata, handles)
% hObject    handle to a_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of a_2 as text
%        str2double(get(hObject,'String')) returns contents of a_2 as a double


% --- Executes during object creation, after setting all properties.
function a_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to a_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in s_1.
function s_1_Callback(hObject, eventdata, handles)
% hObject    handle to s_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of s_1


% --- Executes on button press in s_2.
function s_2_Callback(hObject, eventdata, handles)
% hObject    handle to s_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of s_2


% --- Executes on slider movement.
function steps_Callback(hObject, eventdata, handles)
% hObject    handle to steps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%set(handles.steps, 'Min', 5, 'Max', 25); % Set min to 5, max to 25
%set(handles.steps, 'Value', 5);          % Set the initial value to 5

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function steps_CreateFcn(hObject, eventdata, handles)
% hObject    handle to steps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function speed_Callback(hObject, eventdata, handles)
% hObject    handle to speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pause.
function pause_Callback(hObject, eventdata, handles)
% hObject    handle to pause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%%%%%%%%%%%%%%%%%%%%%%%%%%% VARIABLE ASSIGNMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%
% assigns values entered via GUI to respective variables
a_1 = str2double( get(handles.a_1,'string') ); % link 1 length 
a_2 = str2double( get(handles.a_2,'string') ); % link 2 length
x_i = str2double( get(handles.x_i,'string') ); % initial value of x
y_i = str2double( get(handles.y_i,'string') ); % initial value of y
x_f = str2double( get(handles.x_f,'string') ); % final value of x
y_f = str2double( get(handles.y_f,'string') ); % final value of y
s_1 = get(handles.s_1,'value');                % radio button solution 1 
s_2 = get(handles.s_2,'value');                % radio button solution 2
stp = get(handles.steps,'value');              % slider number of steps
spd = get(handles.speed,'value');              % slider speed
t   = .1/spd;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check LINKS and POINTS informed
if (a_1 < a_2)
    msgbox('Link 2 must be less than or equal to link 1!')
    return
end
if (x_i == 0 && y_i == 0) || (x_f == 0 && y_f == 0)
    msgbox('Can not simulate with start or end point at origin!')
    return
end

% check POINTS whithin the workspace
lim_ext = a_1+a_2;
lim_int = a_1-a_2;
lim_inf = 0;
p_i     = abs( sqrt(x_i^2 +y_i^2) );
p_f     = abs( sqrt(x_f^2 +y_f^2) );
if ( x_i == x_f && y_i == y_f )
    msgbox('Same starting and ending points! There is no movement.')
    return
end
if ( p_i>lim_ext || p_f>lim_ext )
    msgbox('Start or end point outside the workspace!')
    return
end

% linear movement coefficients and increment size
a   = (y_f -y_i)/(x_f -x_i);
b   =  y_i -a*x_i;
inc = (x_f -x_i)/stp;

% check TRAJECTORY within the workspace
for X   = x_i:inc:x_f
    Y   = a*X +b;
    m_p = abs( sqrt(X^2 +Y^2) );
    bt  = linspace(0,180,180);
    
    % plot showing invalid position
    if ( m_p<lim_int || Y<lim_inf )
        cla;
        fill([(a_1+a_2)*cosd(bt) (a_1-a_2)*cosd(bt)],[(a_1+a_2)*sind(bt) (a_1-a_2)*sind(bt)],'m');
        hold on;
        plot([x_i x_f],[y_i y_f],'--or');
        axis off;
        hold off;
        msgbox('Trajectory outside the workspace!');
        return
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solution 1
if s_1 == 1
       
    % straight line equation
    for X = x_i:inc:x_f
        Y = a*X +b;

        % joint angles t_1 and t_2 (inverse kinematics)
        t_2 = acosd( (X^2 +Y^2 -a_1^2 -a_2^2)/(2*a_1*a_2) );
        t_1 = atan2d(Y,X) -atan2d( (a_2*sind(t_2)),(a_1 +a_2*cosd(t_2)) );
        set(handles.t_2,'string',[num2str(round(100*t_2)/100) '°']);
        set(handles.t_1,'string',[num2str(round(100*t_1)/100) '°']);
        
        % calculates points (x_1,y_1) and (x_2,y_2) (direct kinematics)
        x_1 = a_1*cosd(t_1);
        y_1 = a_1*sind(t_1); 
        x_2 = x_1 +a_2*cosd(t_2 +t_1);
        y_2 = y_1 +a_2*sind(t_2 +t_1);
        
        % plot
        if y_1 <0
            msgbox('Unable to execute the path with this solution. Try another solution.')
            return
        end 
        cla;
        fill([(a_1+a_2)*cosd(bt) (a_1-a_2)*cosd(bt)],[(a_1+a_2)*sind(bt) (a_1-a_2)*sind(bt)],'c');
        hold on;        
        plot([x_i   x_f],[y_i   y_f],'--*r');
        plot([0    x_1],[0    y_1],'-ob' ,'linewidth',5,'markersize',8);                   
        plot([x_1 x_2],[y_1 y_2],'-ob','linewidth',5,'markersize',8);
        plot([-(a_1+a_2)-1 (a_1+a_2)+1],[0 0],':k','linewidth',2);
        axis([-(a_1+a_2)-1 (a_1+a_2)+1 (lim_inf-1) (a_1+a_2)+1]);
        ax = gca;
        ax.Layer = 'top';
        grid on;
        pause(t)
        hold off       
    end
end

% solution 2
if s_2 == 1
    
    % straight line equation
    for X = x_i:inc:x_f
        Y = a*X +b;

        % joint angles t_1 and t_2 (inverse kinematics)   
        t_2 = -acosd( (X^2 +Y^2 -a_1^2 -a_2^2)/(2*a_1*a_2) );
        t_1 =  atan2d(Y,X) -atan2d( (a_2*sind(t_2)),(a_1 +a_2*cosd(t_2)) );
        set(handles.t_2,'string',[num2str(round(100*t_2)/100) 'º']);
        set(handles.t_1,'string',[num2str(round(100*t_1)/100) '°']);
        
        % calculates points (x_1,y_1) and (x_2,y_2) (direct kinematics)
        x_1 = a_1*cosd(t_1);
        y_1 = a_1*sind(t_1);
        x_2 = x_1 +a_2*cosd(t_2+t_1);
        y_2 = y_1 +a_2*sind(t_2+t_1);

        % plot
        if y_1 <0
            msgbox('Unable to execute the path with this solution. Try another solution.')
            return
        end 
        cla;
        fill([(a_1+a_2)*cosd(bt) (a_1-a_2)*cosd(bt)],[(a_1+a_2)*sind(bt) (a_1-a_2)*sind(bt)],'c');
        hold on;
        plot([x_i     x_f],[y_i     y_f],'--*r');
        plot([0    x_1],[0    y_1], '-or','linewidth',5,'markersize',8);   
        plot([x_1 x_2],[y_1 y_2], '-or','linewidth',5,'markersize',8);
        plot([-(a_1+a_2)-1 (a_1+a_2)+1],[0 0],':k','linewidth',2);
        axis([-(a_1+a_2)-1 (a_1+a_2)+1 (lim_inf-1) (a_1+a_2)+1]);
        ax = gca;
        ax.Layer = 'top';
        grid on;
        pause(t)
        hold off;
    end
end

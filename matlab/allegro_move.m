function allegro_move
%%
% define the CAN msg struct.
can_msg.STD_EXT = '';
can_msg.msg_id = uint32(1);  % message identifier
can_msg.data_length = uint8(8);
can_msg.data = repmat(uint8(0),8,1);  % data array

% define the Allegro hand CAN vars
max_dofs = 16;
allegro_vars.enc_actual = zeros(max_dofs, 1);
allegro_vars.pwm_actual = zeros(max_dofs, 1);
allegro_vars.pwm_demand = zeros(max_dofs, 1);

% command and feedback vars
q = zeros(max_dofs, 1);
q_des = q;
tau_des = q;
cur_des = q;

% create the CAN channel (specific to PEAK hw).
canch = canChannel('PEAK-System', 'PCAN_USBBUS1'); %disp(canch);
configBusSpeed(canch, 1e6);  % 1 Mbaud/s
start(canch);
if (canch.Running)
    disp(['CAN channel is open and running. Status: ', ...
        num2str(canch.Running)]);
else
    error('CAN channel could not start');
end

% get the available message identifier fields for the Allegro hand.
ids = get_command_ids;

% Set the communication period in milliseconds. This seems to be the 
% minimum achievable period for torque control in Matlab (not great).
period_ms = 10;  
dt = period_ms*1e-3;

CommandSysInit(canch, ids, period_ms);  % initialize and enable the hand

%% Read message tests
%CommandReadState(canch, ids, dt);
%CommandReportMsgCount(canch, dt);  % read the state
%CommandPrintState(canch, ids, 100, 0.75);

%% Send torque commands test

% command only the index finger
%CommandSendIndexTorque(canch, ids, dt);
%CommandSendSingleIndexTorque(canch, ids); pause(0.2);

% command all fingers
%CommandSendSingleTorques(canch, ids, 0.1*ones(16,1)); pause(0.2);
%CommandSendTorques(canch, ids, 0.2*ones(16,1)); pause(0.2);
%CommandSendTorques(canch, ids, zeros(16,1), dt); pause(0.2);

%% Command hand pose test
CommandHandPose(canch, ids, 'zero', dt);
%CommandHandPose(canch, ids, 'mug_grasp', dt);

%% Run motion sequence test
%RunMugTwist(canch, ids, dt);
%RunRockPaperScissors(canch, ids, dt);

%% Stop the process
CommandSysClose(canch, ids);  % disable the hand

disp(canch); % show status of CAN channel before closing
stop(canch);
disp(['CAN channel is closed. Status: ', num2str(canch.Running)]);

clear all;

end

% configures the allegro hand comm period, and cmd mode.
function CommandSysInit(canch, ids, period_ms)

% source and destination ids.
dst_id = ids.ID_COMMON;  % allegro hand id
src_id = ids.ID_DEVICE_MAIN;  % control pc id

% %% command query id (NOT SURE WHAT THIS DOES)
% can_msg.cmd_id = ids.ID_CMD_QUERY_ID;
% can_msg.src_id = src_id;
% can_msg.dst_id = dst_id;
% can_msg.data = 0;
% tx_can_msg(canch, can_msg);

% %% command set AHRS (NOT SURE WHAT THIS DOES)
% can_msg.cmd_id = ids.ID_CMD_AHRS_SET;
% can_msg.src_id = src_id;
% can_msg.dst_id = dst_id;
% rate = hex2dec('04'); mask = bitor(hex2dec('01'), hex2dec('02'));
% can_msg.data = [rate mask]';
% tx_can_msg(canch, can_msg);

%% set the period
can_msg.cmd_id = ids.ID_CMD_SET_PERIOD;
can_msg.src_id = src_id;
can_msg.dst_id = dst_id;
can_msg.data = period_ms;
tx_can_msg(canch, can_msg);

%% set mode joint
can_msg.cmd_id = ids.ID_CMD_SET_MODE_JOINT;
can_msg.src_id = src_id;
can_msg.dst_id = dst_id;
can_msg.data = 0;
tx_can_msg(canch, can_msg);

% %% set cmd query state data (NOT SURE WHAT THIS DOES)
% can_msg.cmd_id = ids.ID_CMD_QUERY_STATE_DATA;
% can_msg.src_id = src_id;
% can_msg.dst_id = dst_id;
% can_msg.data = 0;
% tx_can_msg(canch, can_msg);
% % tx the message twice (NOT SURE WHY)
% pause(10e-6); tx_can_msg(canch, can_msg);


%% set system on
can_msg.cmd_id = ids.ID_CMD_SET_SYSTEM_ON;
can_msg.src_id = src_id;
can_msg.dst_id = dst_id;
can_msg.data = 0;
tx_can_msg(canch, can_msg);

end

function CommandReportMsgCount(canch, dt)
% Read and plot the state for n seconds
sec = 0.1;

close all;
figure('Name', 'Msgs Received'); grid on;

% clear the buffer
msgin = receive(canch, inf);

numindx = ceil(sec/dt);
t = zeros(1, numindx); num = t; count = 0;
for i=1:1e10
    msgin = receive(canch, inf);
    if (~isempty(msgin))
        count = count +1;
        if (length(msgin) == 4)
            msg = msgin(1);
        else
            error('Expected 4 messages, but received more than 4.');
        end
        t(count) = msg.Timestamp; num(count) = length(msgin);
    end
    if (count*dt > sec)
        break;
    end
end

plot(t(1:count), num(1:count), 'k*'); grid on;

disp(['Number of nonempty reads: ', num2str(count)]);

end

function CommandReadState(canch, ids, dt)
% Read and plot the state for n seconds
sec = 10;

close all;
%figure('Name', 'Index_0 position');

% get the finger positions
npoints = 400;
finger_data_points = zeros(npoints,16); % n-by-16
finger_time_points = zeros(npoints,1); % n-by-1 

% initialize, and clear the buffer
t0 = 0; N = length(finger_data_points);
msgs = receive(canch, inf);
t0 = msgs(1).Timestamp;

% get all finger positions, in degrees.
for i=1:N
    [fpos, t] = read_fingers(canch, ids);
    finger_time_points(i) = t - t0;
    fpos = Raw2DegPositions(fpos); % convert to degrees
    finger_data_points(i,:) = fpos;
end

names = {'index', 'middle', 'little', 'thumb'};

for i=1:length(names)
    indx = (i-1)*4 + 1;
    figure('Name',names{i}); plot(finger_time_points, finger_data_points(:,indx:indx+3)); grid on;
    legend('j1','j2','j3','j4');
end

end

function CommandPrintState(canch, ids, s, dt)
% Prints the read joint positions for s seconds every dt seconds

for i=1:ceil(s/dt)
    tic;
    receive(canch, inf); % clear the queue
    fpos = Raw2DegPositions(read_fingers(canch, ids));
    %fpos = Raw2RadPositions(read_fingers(canch, ids));
    %fpos = RawPositions(read_fingers(canch, ids));
    %fpos = read_fingers(canch, ids);
    
    mask = zeros(16,1);
    %mask(1:4) = ones(4,1); % index
    %mask(5:8) = ones(4,1); % middle
    %mask(9:12) = ones(4,1); % little
    %mask(13:16) = ones(4,1); % thumb
    mask(:) = 1; % all
    
    disp(['q = ', mat2str(fpos'.*mask',6)]);
    pause(dt-toc);
end

end

function rawpos = RawPositions(rpos)

% get offsets and directions
enc_offsets = get_enc_offsets;
enc_dirs = get_enc_directions;

rawpos = rpos.*enc_dirs - 32768 - enc_offsets;

end

function dpos = Raw2DegPositions(rpos)
% converts raw sensor positions to degrees, with proper offset and sign.

dpos = RawPositions(rpos)*(333.3/65536.0); % in degrees

end

function radpos = Raw2RadPositions(rpos)
% converts raw sensor positions to radians, with proper offset and sign.

radpos = RawPositions(rpos).*((333.3/65536.0)*(pi/180)); % in radians

end

function CommandSendSingleIndexTorque(canch, ids)

% source and destination ids.
dst_id = ids.ID_COMMON;  % allegro hand id
src_id = ids.ID_DEVICE_MAIN;  % control pc id

%% command a torque to the index finger
% From the documentation:
% Note: PWM = Desired_Torque (N-m) * 1200.0.
% 1200.0 is an empirical constant that will convert torque to PWM.
tau_cov_const_v3 = 1200.0; % torque -> pwm conversion factor

% desired torque is ordered from Distal to Proximal joints (WTF?!).
tau_des = [0 0 0 -0.08];
pwm_des = tau_des .* tau_cov_const_v3; % torque (N-m)->pwm

can_msg.cmd_id = ids.ID_CMD_SET_TORQUE_1;
can_msg.src_id = src_id;
can_msg.dst_id = dst_id;
can_msg.data = zeros(8,1);  % initialize the data vector
can_msg.data([2, 1]) = typecast(int16(pwm_des(1)), 'uint8');
can_msg.data([4, 3]) = typecast(int16(pwm_des(2)), 'uint8');
can_msg.data([6, 5]) = typecast(int16(pwm_des(3)), 'uint8');
can_msg.data([8, 7]) = typecast(int16(pwm_des(4)), 'uint8');

% command torque for 1 second, updating every dt
tx_can_msg(canch, can_msg);

end

function CommandSendIndexTorque(canch, ids, dt)

% command torque for 1 second, updating every dt

for i=1:ceil(1/dt)
    tic;
    CommandSendSingleIndexTorque(canch, ids);
    pause(dt-toc);
end

end

function CommandSendSingleTorques(canch, ids, tau_des)
% Sends a single torque command to each of the joints.
% the size of tau_des must be 16-by-1

assert(length(tau_des) == 16, 'Torque signal must be 16 x 1');

% saturate torques to the range [-1, 1]
tau_des = max(tau_des, -1);
tau_des = min(tau_des, 1);

% source and destination ids.
dst_id = ids.ID_COMMON;  % allegro hand id
src_id = ids.ID_DEVICE_MAIN;  % control pc id

%% command a torque to all fingers
% From the documentation:
% Note: PWM = Desired_Torque (N-m) * 1200.0.
% 1200.0 is an empirical constant that will convert torque to PWM.
tau_cov_const_v3 = 1200.0; % torque -> pwm conversion factor

% desired torque is ordered from Distal to Proximal joints (WTF?!).
pwm_des = tau_des .* tau_cov_const_v3; % torque (N-m)->PWM

for indx=1:4  % loop over all the fingers
    can_msg.cmd_id = ids.ID_CMD_SET_TORQUE_1 + (indx-1);
    can_msg.src_id = src_id;
    can_msg.dst_id = dst_id;
    can_msg.data = zeros(8,1);  % initialize the data vector
    base_indx = (indx-1)*4;
    can_msg.data([2, 1]) = typecast(int16(pwm_des(base_indx + 1)), 'uint8');
    can_msg.data([4, 3]) = typecast(int16(pwm_des(base_indx + 2)), 'uint8');
    can_msg.data([6, 5]) = typecast(int16(pwm_des(base_indx + 3)), 'uint8');
    can_msg.data([8, 7]) = typecast(int16(pwm_des(base_indx + 4)), 'uint8');
    
    tx_can_msg(canch, can_msg);
end

end

function CommandSendTorques(canch, ids, tau_des, dt)
% Sends the same torque commands to all joints for 1 second.

% The size of tau_des must be 16-by-1
assert(length(tau_des) == 16, 'Torque signal must be 16 x 1');

for i=1:ceil(10/dt)
    tic;
    CommandSendSingleTorques(canch, ids, tau_des);
    pause(dt-toc);
end

end

function CommandHandPose(canch, ids, pose_str, dt)

switch pose_str
    case 'paper'
        pose = get_paper_pose;
    case 'rock'
        pose = get_rock_pose;
    case 'scissors'
        pose = get_scissors_pose;
    case 'zero'
        pose = get_zero_pose;
    case 'mug_grasp'
        pose = get_mug_grasp_pose;
    case 'close_thumb'
        pose = get_zero_pose;
        pose(13:14) = [1.396, 0.3]';
    case 'mug_twist1'
        pose = get_mug_grasp_pose;
        offsets = get_twist1_offsets;
        pose = pose + offsets;
    otherwise
        warning(['Pose ',pose_str,' not recognized']);
end

CommandJointPositions(canch, ids, pose, dt);

end

function CommandJointPositions(canch, ids, pos_d, dt)
% Runs a PD control on joint positions for n seconds
secs = 2;
N = ceil(secs/dt); % number of steps

[Kp, Kd] = get_gains(dt*0.15); % get the discretized gains
Kp = Kp.*0.58; Kd = Kd.*0.27;

mask = zeros(16,1); mask(:) = 1; % using position ordering

% implement a control loop to regulate the pose (in position control).
% initialize, and clear the buffer
receive(canch, inf);

% for velocity estimate
pos_last = Raw2RadPositions(read_fingers(canch, ids)); % position actual
vahist = zeros(16,N); % velocity history (rad/s)
vfhist = zeros(16,N); % filtered velocity history
fwindow = 2;

% Finger ordering is [index, middle, little, thumb]
% **Note: position (sensor) ordering is proximal to distal, torque command
%         ordering is distal to proximal!
for i=1:N
    tic;
    [pos_a, ~] = read_fingers(canch, ids); % position actual
    pos_a = Raw2RadPositions(pos_a); % convert to radians
    pos_err = (pos_d - pos_a);
    
    % compute the velocity
    vel_a =  (pos_a - pos_last)./dt;
    vahist(:,i) = vel_a;
    if (i == 1)
        vfhist(:,1) = vel_a;
    else
        indx = i-1:-1:max(1,i-fwindow+1);
        vfhist(:,i) = (vel_a + sum(vfhist(:,indx),2))/(length(indx)+1);
    end
    pos_last = pos_a;
    
    % compute the torque command
    %tau_des = (Kp.*pos_err -Kd.*vel_a).*mask;
    tau_des = (Kp.*pos_err -Kd.*vfhist(:,i)).*mask;
    
    % re-order torques from proximal to distal to distal to proximal
    for j=1:4
        base_indx = (j-1)*4;
        tau_des(base_indx + 4:-1: base_indx + 1) = tau_des(base_indx + 1: base_indx + 4);
    end
    CommandSendSingleTorques(canch, ids, tau_des);
    pause(dt-toc);
end

% % plot the velocity history
% vhist = max(vhist, -10);
% vhist = min(vhist, 10);
%figure(1); plot(0:dt:(N-1)*dt, vahist); grid on; hold on;
%plot(0:dt:(N-1)*dt,vfhist(4,:),'k');

end

function RunMugTwist(canch, ids, dt)
% Run the mug twist task (from drake/examples/allegro_hand)
CommandHandPose(canch, ids, 'zero', dt);
CommandHandPose(canch, ids, 'close_thumb', dt);
CommandHandPose(canch, ids, 'mug_grasp', dt);
CommandHandPose(canch, ids, 'mug_twist1', dt);
CommandHandPose(canch, ids, 'mug_grasp', dt);
CommandHandPose(canch, ids, 'zero', dt);

end

function RunRockPaperScissors(canch, ids, dt)

CommandHandPose(canch, ids, 'zero', dt);
CommandHandPose(canch, ids, 'rock', dt);
CommandHandPose(canch, ids, 'paper', dt);
CommandHandPose(canch, ids, 'scissors', dt);
CommandHandPose(canch, ids, 'zero', dt);

end

function CommandSysClose(canch, ids)
% source and destination ids.
dst_id = ids.ID_COMMON;  % allegro hand id
src_id = ids.ID_DEVICE_MAIN;  % control pc id

%% set system off
can_msg.cmd_id = ids.ID_CMD_SET_SYSTEM_OFF;
can_msg.src_id = src_id;
can_msg.dst_id = dst_id;
can_msg.data = 0;
tx_can_msg(canch, can_msg);

end

function tx_can_msg(canch, can_msg)
% We determine the data length by checking the size of the can_msg.data 
% variable.
% can_msg.cmd_id is the decimal representation of the hex command 
% identifier, e.g., after calling hex2dec.
% can_msg.data is always converted to an array of uint8 in this function,
% regardless of the incoming data type.

cmd_id = can_msg.cmd_id;
src_id = can_msg.src_id;
dst_id = can_msg.dst_id;
data = can_msg.data;

% first pad the data with zeros
dsize = size(data);
if (length(dsize) ~= 2 || (dsize(1) ~= 1 && dsize(2) ~= 1))
    error('Data vector must be an nx1');
end
dlen = length(data);

% make sure data is a column vector
data = reshape(data, dlen, 1);

% front pad the data with zeros to make 8 bytes of data
if (dlen < 8)
    data = [data; zeros(8-dlen,1);];
end

% convert to uint8
data = uint8(data);

% set the command identifier in the CAN msg identifier
cmd_id = bitshift(uint32(cmd_id), 6);

id = bitor(cmd_id, bitshift(dst_id, 3)); % 32-bit CAN message identifier
id = bitor(id, src_id);

message = canMessage(id, false, dlen);

data = typecast(data,'int64');  % typecast to 64-bit integer

pack(message, data, 0, dlen * 8, 'LittleEndian');
transmit(canch, message);

%pause(10*1e-6);

end

function [fpos_out, t] = read_fingers(canch, ids)
%READ_FINGERS Returns an array of 16 raw finger positions.
% [fpos_out, t] = READ_FINGERS(canch, ids) reads on canch with ids.
% The ordering is: [index, middle, little, thumb].
% Joint ordering per finger is proximal to distal.
persistent fpos;

% read the array of finger can messages (4x)
can_msgs = rx_can_msg(canch, 4);

% if we get less than 4 messages, discard all messages and retry.
while (length(can_msgs) < 4)
    can_msgs = rx_can_msg(canch, 4);
end

if (length(can_msgs) ~= 4)
    error(['Expected 4 messages for finger positions, but got ', ...
        num2str(length(can_msgs))]);
end

fpos = zeros(16,1);
t = can_msgs(1).Timestamp;
for i=1:4
   % make sure the message is ID_CMD_QUERY_CONTROL_DATA type
   message = can_msgs(i);
   assert(message.UserData.cmd == ids.ID_CMD_QUERY_CONTROL_DATA, ...
       'Unexpected cmd id type in read_fingers');
   indx = (message.UserData.src - ids.ID_DEVICE_SUB_01)*4; 
   data = message.Data;
   fpos(indx + 1) = typecast(data(1:2),'uint16');
   fpos(indx + 2) = typecast(data(3:4),'uint16');
   fpos(indx + 3) = typecast(data(5:6),'uint16');
   fpos(indx + 4) = typecast(data(7:8),'uint16');
end

fpos_out = fpos;

end

function can_msgs = rx_can_msg(canch, nmsgs)
% A non-blocking read of the CANbus. Returns an array of all can messages
% received, or an empty array if none exist.
% The user data field of the can message contains a struct that defines
% the cmd, src, dest, datalen.

% receive all messages
can_msgs = receive(canch, nmsgs);

% process the message specific info
for i=1:length(can_msgs)
    info.cmd = bitand(bitshift(can_msgs(i).ID, -6), hex2dec('1f'));
    info.dst = bitand(bitshift(can_msgs(i).ID, -3), hex2dec('07'));
    info.src = bitand(can_msgs(i).ID, hex2dec('07'));
    can_msgs(i).UserData = info;
end

end

function ids = get_command_ids

ids.ID_CMD_SET_SYSTEM_ON = hex2dec('01');
ids.ID_CMD_SET_SYSTEM_OFF = hex2dec('02');
ids.ID_CMD_SET_PERIOD = hex2dec('03');
ids.ID_CMD_SET_MODE_JOINT = hex2dec('04');
ids.ID_CMD_SET_MODE_TASK = hex2dec('05');
ids.ID_CMD_SET_TORQUE_1 = hex2dec('06');
ids.ID_CMD_SET_TORQUE_2 = hex2dec('07');
ids.ID_CMD_SET_TORQUE_3 = hex2dec('08');
ids.ID_CMD_SET_TORQUE_4 = hex2dec('09');
ids.ID_CMD_QUERY_STATE_DATA = hex2dec('0e');
ids.ID_CMD_QUERY_CONTROL_DATA = hex2dec('0f');

% These are not documented
ids.ID_CMD_QUERY_ID = hex2dec('10');
ids.ID_CMD_AHRS_SET = hex2dec('11');
ids.ID_CMD_AHRS_POSE = hex2dec('12');
ids.ID_CMD_AHRS_ACC = hex2dec('13');
ids.ID_CMD_AHRS_GYRO = hex2dec('14');
ids.ID_CMD_AHRS_MAG = hex2dec('15');

ids.ID_COMMON = hex2dec('01');
ids.ID_DEVICE_MAIN = hex2dec('02');
ids.ID_DEVICE_SUB_01 = hex2dec('03');
ids.ID_DEVICE_SUB_02 = hex2dec('04');
ids.ID_DEVICE_SUB_03 = hex2dec('05');
ids.ID_DEVICE_SUB_04 = hex2dec('06');

end

function offsets = get_enc_offsets
% SAH030C033R
offsets = [
  -685, -200, -563, -17523,...
  28, -18139, -16958, 414,...
  -228, -456, 444, -522,...
  -368, -439, 30, 17
  ]';
end

function e_directions = get_enc_directions
% SAH030xxxxx
e_directions = [
  1.0, 1.0, 1.0, 1.0,...
  1.0, 1.0, 1.0, 1.0,...
  1.0, 1.0, 1.0, 1.0,...
  1.0, 1.0, 1.0, 1.0
  ]';
end

function m_directions = get_motor_directions
% SAH030xxxxx
m_directions = [
  1.0, 1.0, 1.0, 1.0,...
  1.0, 1.0, 1.0, 1.0,...
  1.0, 1.0, 1.0, 1.0,...
  1.0, 1.0, 1.0, 1.0
  ]';
end

function positions = get_rock_pose
positions = [
    -0.1194, 1.2068, 1.0, 1.4042, ...
    -0.0093, 1.2481, 1.4073, 0.8163, ...
    0.1116, 1.2712, 1.3881, 1.0122, ...
    0.6017, 0.2976, 0.9034, 0.7929
    ]';
end

function positions = get_paper_pose
positions = [
    -0.1220, 0.4, 0.6, -0.0769, ...
	0.0312, 0.4, 0.6, -0.0, ...
	0.1767, 0.4, 0.6, -0.0528, ...
	0.5284, 0.3693, 0.8977, 0.4863
    ]';
end

function positions = get_scissors_pose
positions = [
    0.0885, 0.4, 0.6, -0.0704, ...
	0.0312, 0.4, 0.6, -0.0, ...
	0.1019, 1.2375, 1.1346, 1.0244, ...
	1.0, 0.6331, 1.3509, 1.0
    ]';
end

function positions = get_zero_pose
positions = zeros(16,1);
end

function positions = get_mug_grasp_pose

positions = [
    0.08, 0.9, 0.75, 1.5, ...
    0.1, 0.9, 0.75, 1.5, ...
    0.12, 0.9, 0.75, 1.5, ...
    1.396, 0.85, 0, 1.3
    ]';

end

function offsets = get_twist1_offsets

p1 = 0.6;
p2 = 0.1;

offsets = [
    0, p1*1, p1*0.3, p1*0.5, ... % index
    0, p2*1, p2*1, p2*0.5, ... % middle
    0, 0, 0, 0, ... % little
    0, 0, 0, 0, ... % thumb
    ]';
end

function offsets = get_twist2_offsets

end

function [Kp, Kd] = get_gains(dt)

Kp = [
    500, 800, 400, 500, ...
    500, 800, 400, 500, ...
    500, 800, 400, 500, ...
    600, 500, 600, 600
    ]' * dt;

Kd = [
    25, 50, 55, 40, ...
    25, 50, 55, 40, ...
    25, 50, 55, 40, ...
    50, 50, 50, 40
    ]' * dt;

end
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

% set the communication period.
period_ms = 10;  % ms

CommandSysInit(canch, ids, period_ms);  % initialize and enable the hand
CommandReadState(canch, ids, period_ms*1e-3);
%CommandReportMsgCount(canch, period_ms*1e-3);  % read the state
%CommandSendTorque(canch, ids);  % command torque to the hand
%CommandSendSingleTorque(canch, ids);  % command torque to the hand

CommandSysClose(canch, ids);  % disable the hand

disp(canch); % show status of CAN channel before closing
stop(canch);
disp(['CAN channel is closed. Status: ', num2str(canch.Running)]);

clear all;

end

% configures the allegro hand comm period, and cmd mode.
function CommandSysInit(canch, ids, period_ms)

% source and destination ids.
dst_id = bitshift(ids.ID_COMMON, 3);  % allegro hand id
src_id = uint32(ids.ID_DEVICE_MAIN);  % control pc id

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
npoints = 300;
finger_data_points = zeros(npoints,16);
finger_time_points = zeros(npoints,1);

% get offsets and directions
enc_offsets = get_enc_offsets;
enc_dirs = get_enc_directions;

% clear the buffer
msgs = receive(canch, inf);
t0 = msgs(1).Timestamp;

% get all finger positions, in degrees.
for i=1:length(finger_data_points)
    [fpos, t] = read_fingers(canch, ids);
    finger_time_points(i) = t - t0;
    fpos = (fpos.*enc_dirs-32768-(enc_offsets))*(333.3/65536.0); % in degrees
    finger_data_points(i,:) = fpos;
end

names = {'index', 'middle', 'little', 'thumb'};

%%%% BUG: The thumb joint 1 (base joint) is plotting as joint 2!
for i=1:length(names)
    indx = (i-1)*4 + 1;
    figure('Name',names{i}); plot(finger_time_points, finger_data_points(:,indx:indx+3)); grid on;
    legend('j1','j2','j3','j4');
end

% the mean for each
disp(['mid position mean: ', mat2str(mean(finger_data_points), 1)]);

end

function CommandSendSingleTorque(canch, ids)

% source and destination ids.
dst_id = bitshift(ids.ID_COMMON, 3);  % allegro hand id
src_id = uint32(ids.ID_DEVICE_MAIN);  % control pc id

%% command a torque to the index finger
% From the documentation:
% Note: PWM = Desired_Torque (N-m) * 1200.0.
% 1200.0 is an empirical constant that will convert torque to PWM.
tau_cov_const_v3 = 1200.0; % torque -> pwm conversion factor

% desired torque is ordered from distal to proximal joints.
tau_des = [0 0 0 0.05] .* tau_cov_const_v3; % torque (N-m)->PWM

can_msg.cmd_id = ids.ID_CMD_SET_TORQUE_1;
can_msg.src_id = src_id;
can_msg.dst_id = dst_id;
can_msg.data = zeros(8,1);  % initialize the data vector
can_msg.data([2, 1]) = typecast(int16(tau_des(1)), 'int8');
can_msg.data([4, 3]) = typecast(int16(tau_des(2)), 'int8');
can_msg.data([6, 5]) = typecast(int16(tau_des(3)), 'int8');
can_msg.data([8, 7]) = typecast(int16(tau_des(4)), 'int8');

% command torque for 1 second, updating every dt
tx_can_msg(canch, can_msg);

end

function CommandSendTorque(canch, ids)

% command torque for 1 second, updating every dt
dt = 0.003;
for i=1:ceil(1/dt)
    CommandSendSingleTorque(canch, ids);
    pause(dt);
end

end

function CommandSysClose(canch, ids)
% source and destination ids.
dst_id = bitshift(ids.ID_COMMON, 3);  % allegro hand id
src_id = uint32(ids.ID_DEVICE_MAIN);  % control pc id

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

id = bitor(cmd_id, dst_id); % 32-bit CAN message identifier
id = bitor(id, src_id);

message = canMessage(id, false, dlen);

data = typecast(data,'int64');  % typecast to 64-bit integer

pack(message, data, 0, dlen * 8, 'LittleEndian');
transmit(canch, message);

%pause(10*1e-6);

end

function [fpos, t] = read_fingers(canch, ids)
% Returns an array of 16 finger positions

% read the array of finger can messages (4x)
can_msgs = rx_can_msg(canch);
while (isempty(can_msgs))
    can_msgs = rx_can_msg(canch);
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

end

function can_msgs = rx_can_msg(canch)
% A non-blocking read of the CANbus. Returns an array of all can messages
% received, or an empty array if none exist.
% The user data field of the can message contains a struct that defines
% the cmd, src, dest, datalen.

% receive all messages
can_msgs = receive(canch, inf);

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
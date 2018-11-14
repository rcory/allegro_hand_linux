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

CommandSysInit(canch, 3);  % 3 ms period
stop(canch);

disp(['CAN channel is closed. Status: ', ...
    num2str(canch.Running)]);

clear all;

end

% configures the allegro hand comm period, and cmd mode.
function CommandSysInit(canch, period_ms)

dst_id = bitshift(uint32(hex2dec('01')), 3);  % allegro hand id
src_id = uint32(hex2dec('02'));  % control pc id

%% ========== set the period ==============
cmd_id = bitshift(uint32(hex2dec('03')), 6);  % set period cmd

id = bitor(cmd_id, dst_id); % 32-bit CAN message identifier
id = bitor(id, src_id);

data = uint8(period_ms);  % set the data

message = canMessage(id, false, 1);  % 1 byte for ID_CMD_SET_PERIOD

pack(message, data, 0, 8, 'LittleEndian');
transmit(canch, message);

pause(10*1e-3);

%% =========== set mode joint ===============
cmd_id = bitshift(uint32(hex2dec('04')), 6);  % set mode joint
id = bitor(cmd_id, dst_id); % 32-bit CAN message identifier
id = bitor(id, src_id);

data = uint8(0);

message = canMessage(id, false, 1);  % 0 bytes for ID_CMD_SET_MODE_JOINT

pack(message, data, 0, 8, 'LittleEndian');
transmit(canch, message);

pause(10*1e-3);

%% =========== set system on ===============================
cmd_id = bitshift(uint32(hex2dec('01')), 6);  % set mode joint
id = bitor(cmd_id, dst_id); % 32-bit CAN message identifier
id = bitor(id, src_id);

data = uint8(0);

message = canMessage(id, false, 1);  % 0 bytes

pack(message, data, 0, 8, 'LittleEndian');
transmit(canch, message);

pause(1e-3);

%% =========== command torque to the index finger ==============
tau_cov_const_v3 = 1200.0;
cmd_id = bitshift(uint32(hex2dec('06')), 6);  % set torque index finger
id = bitor(cmd_id, dst_id); % 32-bit CAN message identifier
id = bitor(id, src_id);

% desired current
cur_des = [0 0 0 0.05] .* tau_cov_const_v3; % torque->pwm

data = repmat(uint8(0), 8, 1);  % initialize the data vector
data([2, 1]) = typecast(int16(cur_des(1)), 'int8');
data([4, 3]) = typecast(int16(cur_des(2)), 'int8');
data([6, 5]) = typecast(int16(cur_des(3)), 'int8');
data([8, 7]) = typecast(int16(cur_des(4)), 'int8');

message = canMessage(id, false, 8);  % 8 bytes

data = typecast(data,'int64');  % typecast to 32-bit integer
pack(message, data, 0, 64, 'LittleEndian');

disp('Commanding torque...');
transmit(canch, message);
pause(1);
disp('Done.');

%% =========== set system off ====================================
cmd_id = bitshift(uint32(hex2dec('02')), 6);  % set mode joint
id = bitor(cmd_id, dst_id); % 32-bit CAN message identifier
id = bitor(id, src_id);

data = uint8(0);

message = canMessage(id, false, 1);  % 0 bytes

pack(message, data, 0, 8, 'LittleEndian');
transmit(canch, message);

end
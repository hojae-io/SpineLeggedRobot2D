function out1 = pos_hind_hip(in1,in2)
%POS_HIND_HIP
%    OUT1 = POS_HIND_HIP(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    25-Nov-2024 00:07:54

p_16 = in2(17,:);
q_0 = in1(1,:);
q_1 = in1(2,:);
q_2 = in1(3,:);
out1 = [q_0-p_16.*cos(q_2);q_1-p_16.*sin(q_2)];
end
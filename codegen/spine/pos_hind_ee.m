function out1 = pos_hind_ee(in1,in2)
%POS_HIND_EE
%    OUT1 = POS_HIND_EE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:08

p_16 = in2(17,:);
p_18 = in2(19,:);
p_19 = in2(20,:);
p_20 = in2(21,:);
q_0 = in1(1,:);
q_1 = in1(2,:);
q_2 = in1(3,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
t2 = q_2+q_5;
t3 = cos(t2);
t4 = q_6+t2;
t5 = sin(t2);
out1 = [q_0+p_18.*t5+p_20.*t5-p_16.*cos(q_2)+p_19.*sin(t4);q_1-p_18.*t3-p_20.*t3-p_19.*cos(t4)-p_16.*sin(q_2)];
end
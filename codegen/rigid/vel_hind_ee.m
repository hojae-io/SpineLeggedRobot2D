function out1 = vel_hind_ee(in1,in2)
%VEL_HIND_EE
%    OUT1 = VEL_HIND_EE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    25-Nov-2024 00:07:53

p_16 = in2(17,:);
p_18 = in2(19,:);
p_19 = in2(20,:);
p_20 = in2(21,:);
q_2 = in1(3,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_dot_0 = in1(8,:);
q_dot_1 = in1(9,:);
q_dot_2 = in1(10,:);
q_dot_5 = in1(13,:);
q_dot_6 = in1(14,:);
t2 = q_2+q_5;
t3 = cos(t2);
t4 = q_6+t2;
t5 = sin(t2);
t6 = cos(t4);
t7 = sin(t4);
t8 = p_18.*t3;
t9 = p_20.*t3;
t10 = p_18.*t5;
t11 = p_20.*t5;
t12 = p_19.*t6;
t13 = p_19.*t7;
out1 = [q_dot_0+q_dot_2.*(t8+t9+t12+p_16.*sin(q_2))+q_dot_6.*t12+q_dot_5.*(t8+t9+t12);q_dot_1+q_dot_2.*(t10+t11+t13-p_16.*cos(q_2))+q_dot_6.*t13+q_dot_5.*(t10+t11+t13)];
end
function CoM = CoM_fn(in1,in2)
%CoM_fn
%    CoM = CoM_fn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:08

p_0 = in2(1,:);
p_1 = in2(2,:);
p_2 = in2(3,:);
p_3 = in2(4,:);
p_4 = in2(5,:);
p_12 = in2(13,:);
p_13 = in2(14,:);
p_14 = in2(15,:);
p_15 = in2(16,:);
p_16 = in2(17,:);
p_17 = in2(18,:);
p_18 = in2(19,:);
p_19 = in2(20,:);
q_0 = in1(1,:);
q_1 = in1(2,:);
q_2 = in1(3,:);
q_3 = in1(4,:);
q_4 = in1(5,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_7 = in1(8,:);
t2 = cos(q_2);
t3 = sin(q_2);
t4 = q_2+q_5;
t5 = q_2+q_7;
t6 = p_1.*2.0;
t7 = p_2.*2.0;
t8 = p_3.*2.0;
t9 = p_4.*2.0;
t18 = -q_1;
t10 = p_16.*t2;
t11 = cos(t4);
t12 = cos(t5);
t13 = q_3+t5;
t14 = q_6+t4;
t15 = p_16.*t3;
t16 = sin(t4);
t17 = sin(t5);
t33 = p_0+t6+t7+t8+t9;
t19 = cos(t13);
t20 = cos(t14);
t21 = q_4+t13;
t22 = sin(t13);
t23 = sin(t14);
t24 = p_17.*t11;
t25 = p_16.*t12;
t26 = p_17.*t16;
t27 = p_16.*t17;
t30 = -t10;
t34 = 1.0./t33;
t28 = cos(t21);
t29 = sin(t21);
t31 = p_17.*t19;
t32 = p_17.*t22;
CoM = [t34.*(p_4.*(q_0+t26+t30+p_15.*t16+p_19.*t23)+p_4.*(q_0+t25+t32+p_15.*t22+p_19.*t29)+p_0.*q_0+p_1.*(q_0+t30+p_12.*t16)+p_1.*(q_0+t25+p_12.*t22)+p_2.*(q_0+t30+p_18.*t16+p_13.*t23)+p_2.*(q_0+t25+p_18.*t22+p_13.*t29)+p_3.*(q_0+t26+t30+p_14.*t23)+p_3.*(q_0+t25+t32+p_14.*t29));-t34.*(p_4.*(t15+t18+t24+p_15.*t11+p_19.*t20)-p_3.*(q_1+t27-t31-p_14.*t28)+p_0.*t18-p_1.*(q_1+t27-p_12.*t19)+p_1.*(t15+t18+p_12.*t11)+p_4.*(t18-t27+t31+p_15.*t19+p_19.*t28)-p_2.*(q_1+t27-p_18.*t19-p_13.*t28)+p_2.*(t15+t18+p_18.*t11+p_13.*t20)+p_3.*(t15+t18+t24+p_14.*t20))];
end

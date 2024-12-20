function keypoints = keypoints_fn(in1,in2)
%KEYPOINTS_FN
%    KEYPOINTS = KEYPOINTS_FN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:08

p_16 = in2(17,:);
p_17 = in2(18,:);
p_18 = in2(19,:);
p_19 = in2(20,:);
p_20 = in2(21,:);
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
t6 = p_16.*t2;
t7 = cos(t4);
t8 = cos(t5);
t9 = q_3+t5;
t10 = q_6+t4;
t11 = p_16.*t3;
t12 = sin(t4);
t13 = sin(t5);
t14 = cos(t9);
t15 = cos(t10);
t16 = q_4+t9;
t17 = sin(t9);
t18 = sin(t10);
t19 = p_17.*t7;
t20 = p_16.*t8;
t21 = p_18.*t7;
t22 = p_17.*t12;
t23 = p_16.*t13;
t24 = p_18.*t12;
t27 = -t6;
t28 = -t11;
t25 = cos(t16);
t26 = sin(t16);
t29 = p_17.*t14;
t30 = p_18.*t14;
t31 = p_19.*t15;
t32 = p_17.*t17;
t33 = p_18.*t17;
t34 = p_19.*t18;
t37 = -t19;
t38 = -t21;
t35 = p_19.*t25;
t36 = p_19.*t26;
t39 = -t29;
t40 = -t30;
t41 = -t31;
t42 = -t35;
keypoints = reshape([q_0,q_1,q_0+t20,q_1+t23,q_0+t27,q_1+t28,q_0+t20+t32,q_1+t23+t39,q_0+t20+t33,q_1+t23+t40,q_0+t20+t32+t36,q_1+t23+t39+t42,q_0+t20+t33+t36,q_1+t23+t40+t42,q_0+t20+t33+t36+p_20.*t17,q_1+t23+t40+t42-p_20.*t14,q_0+t22+t27,q_1+t28+t37,q_0+t24+t27,q_1+t28+t38,q_0+t22+t27+t34,q_1+t28+t37+t41,q_0+t24+t27+t34,q_1+t28+t38+t41,q_0+t24+t27+t34+p_20.*t12,q_1+t28+t38+t41-p_20.*t7],[2,13]);
end

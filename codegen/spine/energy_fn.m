function E = energy_fn(in1,in2)
%ENERGY_FN
%    E = ENERGY_FN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:04

p22 = in2(23,:);
p_0 = in2(1,:);
p_1 = in2(2,:);
p_2 = in2(3,:);
p_3 = in2(4,:);
p_4 = in2(5,:);
p_5 = in2(6,:);
p_6 = in2(7,:);
p_7 = in2(8,:);
p_8 = in2(9,:);
p_9 = in2(10,:);
p_10 = in2(11,:);
p_11 = in2(12,:);
p_12 = in2(13,:);
p_13 = in2(14,:);
p_14 = in2(15,:);
p_15 = in2(16,:);
p_16 = in2(17,:);
p_17 = in2(18,:);
p_18 = in2(19,:);
p_19 = in2(20,:);
p_21 = in2(22,:);
q_1 = in1(2,:);
q_2 = in1(3,:);
q_3 = in1(4,:);
q_4 = in1(5,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_7 = in1(8,:);
q_dot_0 = in1(9,:);
q_dot_1 = in1(10,:);
q_dot_2 = in1(11,:);
q_dot_3 = in1(12,:);
q_dot_4 = in1(13,:);
q_dot_5 = in1(14,:);
q_dot_6 = in1(15,:);
q_dot_7 = in1(16,:);
t2 = cos(q_2);
t3 = sin(q_2);
t4 = q_2+q_5;
t5 = q_2+q_7;
t6 = q_dot_2+q_dot_5;
t7 = p_11.^2;
t13 = q_dot_2+q_dot_3+q_dot_7;
t8 = p_16.*t2;
t9 = cos(t4);
t10 = cos(t5);
t11 = q_3+t5;
t12 = q_6+t4;
t14 = q_dot_6+t6;
t15 = p_16.*t3;
t16 = sin(t4);
t17 = sin(t5);
t21 = q_dot_4+t13;
t24 = t6.^2;
t25 = t13.^2;
t18 = cos(t11);
t19 = cos(t12);
t20 = q_4+t11;
t22 = sin(t11);
t23 = sin(t12);
t26 = t14.^2;
t27 = p_15.*t9;
t28 = p_17.*t9;
t29 = p_16.*t10;
t30 = p_18.*t9;
t31 = p_15.*t16;
t32 = p_17.*t16;
t33 = p_16.*t17;
t34 = p_18.*t16;
t37 = -t8;
t38 = t21.^2;
t35 = cos(t20);
t36 = sin(t20);
t39 = p_12.*t18;
t40 = p_13.*t19;
t41 = p_14.*t19;
t42 = p_15.*t18;
t43 = p_17.*t18;
t44 = p_18.*t18;
t45 = p_19.*t19;
t46 = p_12.*t22;
t47 = p_13.*t23;
t48 = p_14.*t23;
t49 = p_15.*t22;
t50 = p_17.*t22;
t51 = p_18.*t22;
t52 = p_19.*t23;
t59 = -t33;
t53 = p_13.*t35;
t54 = p_14.*t35;
t55 = p_19.*t35;
t56 = p_13.*t36;
t57 = p_14.*t36;
t58 = p_19.*t36;
t60 = -t39;
t61 = t29+t46;
t62 = t33+t60;
t63 = t29+t51+t56;
t64 = t29+t50+t57;
t65 = t44+t53+t59;
t66 = t43+t54+t59;
t67 = t29+t49+t50+t58;
t68 = t42+t43+t55+t59;
et1 = p22.*4.3282208e-1+(p_0.*((-q_dot_0+q_dot_2.*t33.*(3.0./5.0)+q_dot_7.*t33.*(3.0./5.0)).^2+(q_dot_1+q_dot_2.*t29.*(3.0./5.0)+q_dot_7.*t29.*(3.0./5.0)).^2))./4.0+(p_0.*((q_dot_1-q_dot_2.*t8.*(3.0./5.0)).^2+(q_dot_0+q_dot_2.*t15.*(3.0./5.0)).^2))./4.0+(p_1.*((q_dot_0+q_dot_3.*t39-q_dot_2.*t62-q_dot_7.*t62).^2+(q_dot_1+q_dot_3.*t46+q_dot_2.*t61+q_dot_7.*t61).^2))./2.0+(p_4.*((q_dot_0+q_dot_2.*(t15+t27+t28+t45)+q_dot_6.*t45+q_dot_5.*(t27+t28+t45)).^2+(q_dot_1+q_dot_2.*(t31+t32+t37+t52)+q_dot_6.*t52+q_dot_5.*(t31+t32+t52)).^2))./2.0+p22.*q_7.*9.304e-1+(p_10.*(q_dot_3+p_11.*q_dot_4).^2)./2.0;
et2 = (p_10.*(q_dot_5+p_11.*q_dot_6).^2)./2.0+(p_6.*t24)./2.0+(p_6.*t25)./2.0+(p_7.*t26)./2.0+(p_9.*t24)./2.0+(p_8.*t26)./2.0+(p_9.*t25)./2.0+(p_7.*t38)./2.0+(p_8.*t38)./2.0+(p_4.*((q_dot_0+q_dot_4.*t55+q_dot_2.*t68+q_dot_7.*t68+q_dot_3.*(t42+t43+t55)).^2+(q_dot_1+q_dot_4.*t58+q_dot_2.*t67+q_dot_7.*t67+q_dot_3.*(t49+t50+t58)).^2))./2.0+(p_2.*((q_dot_0+q_dot_3.*(t44+t53)+q_dot_4.*t53+q_dot_2.*t65+q_dot_7.*t65).^2+(q_dot_1+q_dot_3.*(t51+t56)+q_dot_4.*t56+q_dot_2.*t63+q_dot_7.*t63).^2))./2.0+(p_3.*((q_dot_0+q_dot_3.*(t43+t54)+q_dot_4.*t54+q_dot_2.*t66+q_dot_7.*t66).^2+(q_dot_1+q_dot_3.*(t50+t57)+q_dot_4.*t57+q_dot_2.*t64+q_dot_7.*t64).^2))./2.0;
et3 = (p_3.*((q_dot_0+q_dot_5.*(t28+t41)+q_dot_6.*t41+q_dot_2.*(t15+t28+t41)).^2+(q_dot_1+q_dot_5.*(t32+t48)+q_dot_6.*t48+q_dot_2.*(t32+t37+t48)).^2))./2.0+(p_2.*((q_dot_0+q_dot_5.*(t30+t40)+q_dot_6.*t40+q_dot_2.*(t15+t30+t40)).^2+(q_dot_1+q_dot_5.*(t34+t47)+q_dot_6.*t47+q_dot_2.*(t34+t37+t47)).^2))./2.0+(p_5.*(q_dot_2+q_dot_7).^2)./1.6e+1+(p22.*q_7.^2)./2.0+(p_5.*q_dot_2.^2)./1.6e+1+(p_1.*((q_dot_0+q_dot_2.*(t15+p_12.*t9)+p_12.*q_dot_5.*t9).^2+(q_dot_1-q_dot_2.*(t8-p_12.*t16)+p_12.*q_dot_5.*t16).^2))./2.0+(p_10.*q_dot_3.^2.*t7)./2.0+(p_10.*q_dot_5.^2.*t7)./2.0+p_0.*p_21.*q_1+p_1.*p_21.*q_1.*2.0+p_2.*p_21.*q_1.*2.0+p_3.*p_21.*q_1.*2.0+p_4.*p_21.*q_1.*2.0;
et4 = p_0.*p_21.*t15.*(-3.0./1.0e+1)-p_1.*p_21.*t15-p_2.*p_21.*t15-p_3.*p_21.*t15-p_4.*p_21.*t15-p_3.*p_21.*t28-p_4.*p_21.*t27-p_2.*p_21.*t30-p_4.*p_21.*t28+p_0.*p_21.*t33.*(3.0./1.0e+1)+p_1.*p_21.*t33+p_2.*p_21.*t33+p_3.*p_21.*t33+p_4.*p_21.*t33-p_2.*p_21.*t40-p_3.*p_21.*t41-p_2.*p_21.*t44-p_3.*p_21.*t43-p_4.*p_21.*t42-p_4.*p_21.*t43-p_4.*p_21.*t45-p_2.*p_21.*t53-p_3.*p_21.*t54-p_4.*p_21.*t55+p_1.*p_21.*t60-p_1.*p_12.*p_21.*t9;
E = et1+et2+et3+et4;
end

function C = C_fn(in1,in2)
%C_fn
%    C = C_fn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    25-Nov-2024 00:07:51

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
q_2 = in1(3,:);
q_3 = in1(4,:);
q_4 = in1(5,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_dot_2 = in1(10,:);
q_dot_3 = in1(11,:);
q_dot_4 = in1(12,:);
q_dot_5 = in1(13,:);
q_dot_6 = in1(14,:);
t2 = cos(q_3);
t3 = cos(q_5);
t4 = sin(q_4);
t5 = sin(q_6);
t6 = q_2+q_3;
t7 = q_2+q_5;
t8 = q_3+q_4;
t9 = q_5+q_6;
t10 = q_dot_2.^2;
t11 = q_dot_3.^2;
t12 = q_dot_4.^2;
t13 = q_dot_5.^2;
t14 = q_dot_6.^2;
t15 = cos(t6);
t16 = cos(t7);
t17 = cos(t8);
t18 = cos(t9);
t19 = q_4+t6;
t20 = q_6+t7;
t21 = sin(t6);
t22 = sin(t7);
t27 = p_2.*p_13.*p_18.*t4.*t12;
t28 = p_3.*p_14.*p_17.*t4.*t12;
t29 = p_2.*p_13.*p_18.*t5.*t14;
t30 = p_3.*p_14.*p_17.*t5.*t14;
t31 = p_4.*p_15.*p_19.*t4.*t12;
t32 = p_4.*p_17.*p_19.*t4.*t12;
t33 = p_4.*p_15.*p_19.*t5.*t14;
t34 = p_4.*p_17.*p_19.*t5.*t14;
t35 = p_2.*p_13.*p_18.*q_dot_2.*q_dot_4.*t4.*2.0;
t36 = p_2.*p_13.*p_18.*q_dot_3.*q_dot_4.*t4.*2.0;
t37 = p_3.*p_14.*p_17.*q_dot_2.*q_dot_4.*t4.*2.0;
t38 = p_3.*p_14.*p_17.*q_dot_3.*q_dot_4.*t4.*2.0;
t39 = p_2.*p_13.*p_18.*q_dot_2.*q_dot_6.*t5.*2.0;
t40 = p_3.*p_14.*p_17.*q_dot_2.*q_dot_6.*t5.*2.0;
t41 = p_4.*p_15.*p_19.*q_dot_2.*q_dot_4.*t4.*2.0;
t42 = p_4.*p_15.*p_19.*q_dot_3.*q_dot_4.*t4.*2.0;
t43 = p_2.*p_13.*p_18.*q_dot_5.*q_dot_6.*t5.*2.0;
t44 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_4.*t4.*2.0;
t45 = p_3.*p_14.*p_17.*q_dot_5.*q_dot_6.*t5.*2.0;
t46 = p_4.*p_17.*p_19.*q_dot_3.*q_dot_4.*t4.*2.0;
t47 = p_4.*p_15.*p_19.*q_dot_2.*q_dot_6.*t5.*2.0;
t48 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_6.*t5.*2.0;
t49 = p_4.*p_15.*p_19.*q_dot_5.*q_dot_6.*t5.*2.0;
t50 = p_4.*p_17.*p_19.*q_dot_5.*q_dot_6.*t5.*2.0;
t23 = cos(t19);
t24 = cos(t20);
t25 = sin(t19);
t26 = sin(t20);
t51 = -t35;
t52 = -t36;
t53 = -t37;
t54 = -t38;
t55 = -t39;
t56 = -t40;
t57 = -t41;
t58 = -t42;
t59 = -t43;
t60 = -t44;
t61 = -t45;
t62 = -t46;
t63 = -t47;
t64 = -t48;
t65 = -t49;
t66 = -t50;
t67 = p_2.*p_13.*p_16.*t10.*t17;
t68 = p_3.*p_14.*p_16.*t10.*t17;
t69 = p_2.*p_13.*p_16.*t10.*t18;
t70 = p_3.*p_14.*p_16.*t10.*t18;
t71 = p_4.*p_16.*p_19.*t10.*t17;
t72 = p_4.*p_16.*p_19.*t10.*t18;
t73 = -t27;
t74 = -t28;
t75 = -t29;
t76 = -t30;
t77 = -t31;
t78 = -t32;
t79 = -t33;
t80 = -t34;
t81 = -t67;
t82 = -t68;
t83 = -t71;
et1 = -p_1.*p_12.*t10.*t21-p_1.*p_12.*t10.*t22-p_1.*p_12.*t11.*t21-p_1.*p_12.*t13.*t22-p_2.*p_13.*t10.*t25-p_4.*p_15.*t10.*t21-p_2.*p_13.*t10.*t26-p_2.*p_13.*t11.*t25-p_2.*p_18.*t10.*t21-p_3.*p_17.*t10.*t21-p_4.*p_15.*t10.*t22-p_4.*p_15.*t11.*t21-p_2.*p_13.*t12.*t25-p_2.*p_18.*t10.*t22-p_2.*p_18.*t11.*t21-p_3.*p_14.*t10.*t25-p_3.*p_17.*t10.*t22-p_3.*p_17.*t11.*t21-p_4.*p_17.*t10.*t21-p_3.*p_14.*t10.*t26-p_3.*p_14.*t11.*t25-p_4.*p_17.*t10.*t22-p_4.*p_17.*t11.*t21-p_2.*p_13.*t13.*t26-p_3.*p_14.*t12.*t25-p_4.*p_15.*t13.*t22-p_2.*p_13.*t14.*t26-p_2.*p_18.*t13.*t22;
et2 = -p_3.*p_17.*t13.*t22-p_3.*p_14.*t13.*t26-p_4.*p_17.*t13.*t22-p_3.*p_14.*t14.*t26-p_4.*p_19.*t10.*t25-p_4.*p_19.*t10.*t26-p_4.*p_19.*t11.*t25-p_4.*p_19.*t12.*t25-p_4.*p_19.*t13.*t26-p_4.*p_19.*t14.*t26-p_1.*p_12.*q_dot_2.*q_dot_3.*t21.*2.0-p_1.*p_12.*q_dot_2.*q_dot_5.*t22.*2.0-p_2.*p_13.*q_dot_2.*q_dot_3.*t25.*2.0-p_4.*p_15.*q_dot_2.*q_dot_3.*t21.*2.0-p_2.*p_13.*q_dot_2.*q_dot_4.*t25.*2.0-p_2.*p_18.*q_dot_2.*q_dot_3.*t21.*2.0-p_3.*p_17.*q_dot_2.*q_dot_3.*t21.*2.0-p_2.*p_13.*q_dot_3.*q_dot_4.*t25.*2.0-p_3.*p_14.*q_dot_2.*q_dot_3.*t25.*2.0-p_4.*p_17.*q_dot_2.*q_dot_3.*t21.*2.0-p_2.*p_13.*q_dot_2.*q_dot_5.*t26.*2.0-p_3.*p_14.*q_dot_2.*q_dot_4.*t25.*2.0-p_4.*p_15.*q_dot_2.*q_dot_5.*t22.*2.0-p_2.*p_13.*q_dot_2.*q_dot_6.*t26.*2.0-p_2.*p_18.*q_dot_2.*q_dot_5.*t22.*2.0-p_3.*p_14.*q_dot_3.*q_dot_4.*t25.*2.0;
et3 = p_3.*p_17.*q_dot_2.*q_dot_5.*t22.*-2.0-p_3.*p_14.*q_dot_2.*q_dot_5.*t26.*2.0-p_4.*p_17.*q_dot_2.*q_dot_5.*t22.*2.0-p_3.*p_14.*q_dot_2.*q_dot_6.*t26.*2.0-p_2.*p_13.*q_dot_5.*q_dot_6.*t26.*2.0-p_4.*p_19.*q_dot_2.*q_dot_3.*t25.*2.0-p_3.*p_14.*q_dot_5.*q_dot_6.*t26.*2.0-p_4.*p_19.*q_dot_2.*q_dot_4.*t25.*2.0-p_4.*p_19.*q_dot_3.*q_dot_4.*t25.*2.0-p_4.*p_19.*q_dot_2.*q_dot_5.*t26.*2.0-p_4.*p_19.*q_dot_2.*q_dot_6.*t26.*2.0-p_4.*p_19.*q_dot_5.*q_dot_6.*t26.*2.0;
et4 = p_1.*p_12.*t10.*t15+p_1.*p_12.*t10.*t16+p_1.*p_12.*t11.*t15+p_1.*p_12.*t13.*t16+p_4.*p_15.*t10.*t15+p_2.*p_18.*t10.*t15+p_3.*p_17.*t10.*t15+p_4.*p_15.*t10.*t16+p_4.*p_15.*t11.*t15+p_2.*p_18.*t10.*t16+p_2.*p_18.*t11.*t15+p_3.*p_17.*t10.*t16+p_3.*p_17.*t11.*t15+p_4.*p_17.*t10.*t15+p_4.*p_17.*t10.*t16+p_4.*p_17.*t11.*t15+p_2.*p_13.*t10.*t23+p_4.*p_15.*t13.*t16+p_2.*p_13.*t10.*t24+p_2.*p_13.*t11.*t23+p_2.*p_18.*t13.*t16+p_3.*p_17.*t13.*t16+p_2.*p_13.*t12.*t23+p_3.*p_14.*t10.*t23+p_4.*p_17.*t13.*t16+p_3.*p_14.*t10.*t24+p_3.*p_14.*t11.*t23+p_2.*p_13.*t13.*t24+p_3.*p_14.*t12.*t23+p_2.*p_13.*t14.*t24+p_3.*p_14.*t13.*t24+p_3.*p_14.*t14.*t24+p_4.*p_19.*t10.*t23+p_4.*p_19.*t10.*t24+p_4.*p_19.*t11.*t23+p_4.*p_19.*t12.*t23+p_4.*p_19.*t13.*t24+p_4.*p_19.*t14.*t24+p_1.*p_12.*q_dot_2.*q_dot_3.*t15.*2.0;
et5 = p_1.*p_12.*q_dot_2.*q_dot_5.*t16.*2.0+p_4.*p_15.*q_dot_2.*q_dot_3.*t15.*2.0+p_2.*p_18.*q_dot_2.*q_dot_3.*t15.*2.0+p_3.*p_17.*q_dot_2.*q_dot_3.*t15.*2.0+p_4.*p_17.*q_dot_2.*q_dot_3.*t15.*2.0+p_4.*p_15.*q_dot_2.*q_dot_5.*t16.*2.0+p_2.*p_13.*q_dot_2.*q_dot_3.*t23.*2.0+p_2.*p_18.*q_dot_2.*q_dot_5.*t16.*2.0+p_3.*p_17.*q_dot_2.*q_dot_5.*t16.*2.0+p_2.*p_13.*q_dot_2.*q_dot_4.*t23.*2.0+p_4.*p_17.*q_dot_2.*q_dot_5.*t16.*2.0+p_2.*p_13.*q_dot_3.*q_dot_4.*t23.*2.0+p_3.*p_14.*q_dot_2.*q_dot_3.*t23.*2.0+p_2.*p_13.*q_dot_2.*q_dot_5.*t24.*2.0+p_3.*p_14.*q_dot_2.*q_dot_4.*t23.*2.0+p_2.*p_13.*q_dot_2.*q_dot_6.*t24.*2.0+p_3.*p_14.*q_dot_3.*q_dot_4.*t23.*2.0+p_3.*p_14.*q_dot_2.*q_dot_5.*t24.*2.0+p_3.*p_14.*q_dot_2.*q_dot_6.*t24.*2.0+p_2.*p_13.*q_dot_5.*q_dot_6.*t24.*2.0+p_4.*p_19.*q_dot_2.*q_dot_3.*t23.*2.0+p_3.*p_14.*q_dot_5.*q_dot_6.*t24.*2.0+p_4.*p_19.*q_dot_2.*q_dot_4.*t23.*2.0+p_4.*p_19.*q_dot_3.*q_dot_4.*t23.*2.0+p_4.*p_19.*q_dot_2.*q_dot_5.*t24.*2.0+p_4.*p_19.*q_dot_2.*q_dot_6.*t24.*2.0+p_4.*p_19.*q_dot_5.*q_dot_6.*t24.*2.0;
et6 = t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65+t66+t73+t74+t75+t76+t77+t78+t79+t80+p_1.*p_12.*p_16.*t2.*t11-p_1.*p_12.*p_16.*t3.*t13+p_4.*p_15.*p_16.*t2.*t11+p_2.*p_16.*p_18.*t2.*t11+p_3.*p_16.*p_17.*t2.*t11+p_4.*p_16.*p_17.*t2.*t11-p_4.*p_15.*p_16.*t3.*t13-p_2.*p_16.*p_18.*t3.*t13-p_3.*p_16.*p_17.*t3.*t13-p_4.*p_16.*p_17.*t3.*t13+p_2.*p_13.*p_16.*t11.*t17+p_2.*p_13.*p_16.*t12.*t17+p_3.*p_14.*p_16.*t11.*t17-p_2.*p_13.*p_16.*t13.*t18+p_3.*p_14.*p_16.*t12.*t17-p_2.*p_13.*p_16.*t14.*t18-p_3.*p_14.*p_16.*t13.*t18-p_3.*p_14.*p_16.*t14.*t18+p_4.*p_16.*p_19.*t11.*t17+p_4.*p_16.*p_19.*t12.*t17-p_4.*p_16.*p_19.*t13.*t18-p_4.*p_16.*p_19.*t14.*t18+p_1.*p_12.*p_16.*q_dot_2.*q_dot_3.*t2.*2.0-p_1.*p_12.*p_16.*q_dot_2.*q_dot_5.*t3.*2.0;
et7 = p_4.*p_15.*p_16.*q_dot_2.*q_dot_3.*t2.*2.0+p_2.*p_16.*p_18.*q_dot_2.*q_dot_3.*t2.*2.0+p_3.*p_16.*p_17.*q_dot_2.*q_dot_3.*t2.*2.0+p_4.*p_16.*p_17.*q_dot_2.*q_dot_3.*t2.*2.0-p_4.*p_15.*p_16.*q_dot_2.*q_dot_5.*t3.*2.0-p_2.*p_16.*p_18.*q_dot_2.*q_dot_5.*t3.*2.0-p_3.*p_16.*p_17.*q_dot_2.*q_dot_5.*t3.*2.0-p_4.*p_16.*p_17.*q_dot_2.*q_dot_5.*t3.*2.0+p_2.*p_13.*p_16.*q_dot_2.*q_dot_3.*t17.*2.0+p_2.*p_13.*p_16.*q_dot_2.*q_dot_4.*t17.*2.0+p_2.*p_13.*p_16.*q_dot_3.*q_dot_4.*t17.*2.0+p_3.*p_14.*p_16.*q_dot_2.*q_dot_3.*t17.*2.0-p_2.*p_13.*p_16.*q_dot_2.*q_dot_5.*t18.*2.0+p_3.*p_14.*p_16.*q_dot_2.*q_dot_4.*t17.*2.0-p_2.*p_13.*p_16.*q_dot_2.*q_dot_6.*t18.*2.0+p_3.*p_14.*p_16.*q_dot_3.*q_dot_4.*t17.*2.0-p_3.*p_14.*p_16.*q_dot_2.*q_dot_5.*t18.*2.0-p_3.*p_14.*p_16.*q_dot_2.*q_dot_6.*t18.*2.0-p_2.*p_13.*p_16.*q_dot_5.*q_dot_6.*t18.*2.0+p_4.*p_16.*p_19.*q_dot_2.*q_dot_3.*t17.*2.0-p_3.*p_14.*p_16.*q_dot_5.*q_dot_6.*t18.*2.0+p_4.*p_16.*p_19.*q_dot_2.*q_dot_4.*t17.*2.0+p_4.*p_16.*p_19.*q_dot_3.*q_dot_4.*t17.*2.0;
et8 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_5.*t18.*-2.0-p_4.*p_16.*p_19.*q_dot_2.*q_dot_6.*t18.*2.0-p_4.*p_16.*p_19.*q_dot_5.*q_dot_6.*t18.*2.0;
mt1 = [et1+et2+et3;et4+et5;et6+et7+et8;t51+t52+t53+t54+t57+t58+t60+t62+t73+t74+t77+t78+t81+t82+t83-p_1.*p_12.*p_16.*t2.*t10-p_4.*p_15.*p_16.*t2.*t10-p_2.*p_16.*p_18.*t2.*t10-p_3.*p_16.*p_17.*t2.*t10-p_4.*p_16.*p_17.*t2.*t10;t81+t82+t83+p_2.*p_13.*p_18.*t4.*t10+p_2.*p_13.*p_18.*t4.*t11+p_3.*p_14.*p_17.*t4.*t10+p_3.*p_14.*p_17.*t4.*t11+p_4.*p_15.*p_19.*t4.*t10+p_4.*p_15.*p_19.*t4.*t11+p_4.*p_17.*p_19.*t4.*t10+p_4.*p_17.*p_19.*t4.*t11+p_2.*p_13.*p_18.*q_dot_2.*q_dot_3.*t4.*2.0+p_3.*p_14.*p_17.*q_dot_2.*q_dot_3.*t4.*2.0+p_4.*p_15.*p_19.*q_dot_2.*q_dot_3.*t4.*2.0+p_4.*p_17.*p_19.*q_dot_2.*q_dot_3.*t4.*2.0;t55+t56+t59+t61+t63+t64+t65+t66+t69+t70+t72+t75+t76+t79+t80+p_1.*p_12.*p_16.*t3.*t10+p_4.*p_15.*p_16.*t3.*t10+p_2.*p_16.*p_18.*t3.*t10+p_3.*p_16.*p_17.*t3.*t10+p_4.*p_16.*p_17.*t3.*t10];
mt2 = [t69+t70+t72+p_2.*p_13.*p_18.*t5.*t10+p_3.*p_14.*p_17.*t5.*t10+p_2.*p_13.*p_18.*t5.*t13+p_3.*p_14.*p_17.*t5.*t13+p_4.*p_15.*p_19.*t5.*t10+p_4.*p_17.*p_19.*t5.*t10+p_4.*p_15.*p_19.*t5.*t13+p_4.*p_17.*p_19.*t5.*t13+p_2.*p_13.*p_18.*q_dot_2.*q_dot_5.*t5.*2.0+p_3.*p_14.*p_17.*q_dot_2.*q_dot_5.*t5.*2.0+p_4.*p_15.*p_19.*q_dot_2.*q_dot_5.*t5.*2.0+p_4.*p_17.*p_19.*q_dot_2.*q_dot_5.*t5.*2.0];
C = [mt1;mt2];
end

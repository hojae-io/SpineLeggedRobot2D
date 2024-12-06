function C = C_fn(in1,in2)
%C_fn
%    C = C_fn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:04

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
q_2 = in1(3,:);
q_3 = in1(4,:);
q_4 = in1(5,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_7 = in1(8,:);
q_dot_2 = in1(11,:);
q_dot_3 = in1(12,:);
q_dot_4 = in1(13,:);
q_dot_5 = in1(14,:);
q_dot_6 = in1(15,:);
q_dot_7 = in1(16,:);
t2 = cos(q_2);
t3 = cos(q_3);
t4 = cos(q_5);
t5 = sin(q_2);
t6 = sin(q_4);
t7 = sin(q_6);
t8 = q_2+q_5;
t9 = q_3+q_4;
t10 = q_2+q_7;
t11 = q_5+q_6;
t12 = q_dot_2.^2;
t13 = q_dot_3.^2;
t14 = q_dot_4.^2;
t15 = q_dot_5.^2;
t16 = q_dot_6.^2;
t17 = q_dot_7.^2;
t18 = cos(t8);
t19 = cos(t9);
t20 = cos(t10);
t21 = cos(t11);
t22 = q_3+t10;
t23 = q_6+t8;
t24 = sin(t8);
t25 = sin(t10);
t28 = t9+t10;
t33 = p_1.*p_12.*p_16.*t3.*t13;
t34 = p_4.*p_15.*p_16.*t3.*t13;
t35 = p_2.*p_16.*p_18.*t3.*t13;
t36 = p_3.*p_16.*p_17.*t3.*t13;
t37 = p_4.*p_16.*p_17.*t3.*t13;
t38 = p_2.*p_13.*p_18.*t6.*t14;
t39 = p_3.*p_14.*p_17.*t6.*t14;
t40 = p_2.*p_13.*p_18.*t7.*t16;
t41 = p_3.*p_14.*p_17.*t7.*t16;
t42 = p_4.*p_15.*p_19.*t6.*t14;
t43 = p_4.*p_17.*p_19.*t6.*t14;
t44 = p_4.*p_15.*p_19.*t7.*t16;
t45 = p_4.*p_17.*p_19.*t7.*t16;
t46 = p_1.*p_12.*p_16.*q_dot_2.*q_dot_3.*t3.*2.0;
t47 = p_1.*p_12.*p_16.*q_dot_3.*q_dot_7.*t3.*2.0;
t48 = p_4.*p_15.*p_16.*q_dot_2.*q_dot_3.*t3.*2.0;
t49 = p_2.*p_16.*p_18.*q_dot_2.*q_dot_3.*t3.*2.0;
t50 = p_3.*p_16.*p_17.*q_dot_2.*q_dot_3.*t3.*2.0;
t51 = p_4.*p_16.*p_17.*q_dot_2.*q_dot_3.*t3.*2.0;
t52 = p_4.*p_15.*p_16.*q_dot_3.*q_dot_7.*t3.*2.0;
t53 = p_2.*p_16.*p_18.*q_dot_3.*q_dot_7.*t3.*2.0;
t54 = p_3.*p_16.*p_17.*q_dot_3.*q_dot_7.*t3.*2.0;
t55 = p_4.*p_16.*p_17.*q_dot_3.*q_dot_7.*t3.*2.0;
t56 = p_2.*p_13.*p_18.*q_dot_2.*q_dot_4.*t6.*2.0;
t57 = p_2.*p_13.*p_18.*q_dot_3.*q_dot_4.*t6.*2.0;
t58 = p_3.*p_14.*p_17.*q_dot_2.*q_dot_4.*t6.*2.0;
t59 = p_3.*p_14.*p_17.*q_dot_3.*q_dot_4.*t6.*2.0;
t60 = p_2.*p_13.*p_18.*q_dot_2.*q_dot_6.*t7.*2.0;
t61 = p_2.*p_13.*p_18.*q_dot_4.*q_dot_7.*t6.*2.0;
t62 = p_3.*p_14.*p_17.*q_dot_2.*q_dot_6.*t7.*2.0;
t63 = p_4.*p_15.*p_19.*q_dot_2.*q_dot_4.*t6.*2.0;
t64 = p_3.*p_14.*p_17.*q_dot_4.*q_dot_7.*t6.*2.0;
t65 = p_4.*p_15.*p_19.*q_dot_3.*q_dot_4.*t6.*2.0;
t66 = p_2.*p_13.*p_18.*q_dot_5.*q_dot_6.*t7.*2.0;
t67 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_4.*t6.*2.0;
t68 = p_3.*p_14.*p_17.*q_dot_5.*q_dot_6.*t7.*2.0;
t69 = p_4.*p_17.*p_19.*q_dot_3.*q_dot_4.*t6.*2.0;
t70 = p_4.*p_15.*p_19.*q_dot_2.*q_dot_6.*t7.*2.0;
t71 = p_4.*p_15.*p_19.*q_dot_4.*q_dot_7.*t6.*2.0;
t72 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_6.*t7.*2.0;
t73 = p_4.*p_15.*p_19.*q_dot_5.*q_dot_6.*t7.*2.0;
t74 = p_4.*p_17.*p_19.*q_dot_4.*q_dot_7.*t6.*2.0;
t75 = p_4.*p_17.*p_19.*q_dot_5.*q_dot_6.*t7.*2.0;
t26 = cos(t22);
t27 = cos(t23);
t29 = sin(t22);
t30 = sin(t23);
t31 = cos(t28);
t32 = sin(t28);
t76 = -t56;
t77 = -t57;
t78 = -t58;
t79 = -t59;
t80 = -t60;
t81 = -t61;
t82 = -t62;
t83 = -t63;
t84 = -t64;
t85 = -t65;
t86 = -t66;
t87 = -t67;
t88 = -t68;
t89 = -t69;
t90 = -t70;
t91 = -t71;
t92 = -t72;
t93 = -t73;
t94 = -t74;
t95 = -t75;
t96 = p_2.*p_13.*p_16.*t12.*t19;
t97 = p_2.*p_13.*p_16.*t13.*t19;
t98 = p_2.*p_13.*p_16.*t14.*t19;
t99 = p_3.*p_14.*p_16.*t12.*t19;
t100 = p_3.*p_14.*p_16.*t13.*t19;
t101 = p_2.*p_13.*p_16.*t12.*t21;
t102 = p_3.*p_14.*p_16.*t14.*t19;
t103 = p_2.*p_13.*p_16.*t17.*t19;
t104 = p_3.*p_14.*p_16.*t12.*t21;
t105 = p_3.*p_14.*p_16.*t17.*t19;
t106 = p_4.*p_16.*p_19.*t12.*t19;
t107 = p_4.*p_16.*p_19.*t13.*t19;
t108 = p_4.*p_16.*p_19.*t14.*t19;
t109 = p_4.*p_16.*p_19.*t12.*t21;
t110 = p_4.*p_16.*p_19.*t17.*t19;
t111 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_3.*t19.*2.0;
t112 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_4.*t19.*2.0;
t113 = p_2.*p_13.*p_16.*q_dot_3.*q_dot_4.*t19.*2.0;
t114 = p_3.*p_14.*p_16.*q_dot_2.*q_dot_3.*t19.*2.0;
t115 = p_3.*p_14.*p_16.*q_dot_2.*q_dot_4.*t19.*2.0;
t116 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_7.*t19.*2.0;
t117 = p_3.*p_14.*p_16.*q_dot_3.*q_dot_4.*t19.*2.0;
t118 = p_2.*p_13.*p_16.*q_dot_3.*q_dot_7.*t19.*2.0;
t119 = p_2.*p_13.*p_16.*q_dot_4.*q_dot_7.*t19.*2.0;
t120 = p_3.*p_14.*p_16.*q_dot_2.*q_dot_7.*t19.*2.0;
t121 = p_3.*p_14.*p_16.*q_dot_3.*q_dot_7.*t19.*2.0;
t122 = p_3.*p_14.*p_16.*q_dot_4.*q_dot_7.*t19.*2.0;
t123 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_3.*t19.*2.0;
t124 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_4.*t19.*2.0;
t125 = p_4.*p_16.*p_19.*q_dot_3.*q_dot_4.*t19.*2.0;
t126 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_7.*t19.*2.0;
t127 = p_4.*p_16.*p_19.*q_dot_3.*q_dot_7.*t19.*2.0;
t128 = p_4.*p_16.*p_19.*q_dot_4.*q_dot_7.*t19.*2.0;
t129 = -t38;
t130 = -t39;
t131 = -t40;
t132 = -t41;
t133 = -t42;
t134 = -t43;
t135 = -t44;
t136 = -t45;
t137 = -t116;
t138 = -t120;
t139 = -t126;
t140 = -t96;
t141 = -t99;
t142 = -t103;
t143 = -t105;
t144 = -t106;
t145 = -t110;
et1 = p_0.*p_16.*t2.*t12.*(3.0./1.0e+1)+p_1.*p_16.*t2.*t12+p_2.*p_16.*t2.*t12+p_3.*p_16.*t2.*t12+p_4.*p_16.*t2.*t12-p_0.*p_16.*t12.*t20.*(3.0./1.0e+1)-p_1.*p_12.*t12.*t24-p_1.*p_16.*t12.*t20-p_2.*p_16.*t12.*t20-p_3.*p_16.*t12.*t20-p_1.*p_12.*t15.*t24-p_4.*p_16.*t12.*t20-p_0.*p_16.*t17.*t20.*(3.0./1.0e+1)-p_1.*p_12.*t12.*t29-p_1.*p_16.*t17.*t20-p_1.*p_12.*t13.*t29-p_2.*p_16.*t17.*t20-p_4.*p_15.*t12.*t24-p_2.*p_18.*t12.*t24-p_3.*p_16.*t17.*t20-p_3.*p_17.*t12.*t24-p_2.*p_13.*t12.*t30-p_4.*p_16.*t17.*t20-p_4.*p_17.*t12.*t24-p_4.*p_15.*t15.*t24-p_1.*p_12.*t17.*t29-p_2.*p_13.*t12.*t32-p_2.*p_18.*t15.*t24;
et2 = -p_3.*p_14.*t12.*t30-p_3.*p_17.*t15.*t24-p_2.*p_13.*t13.*t32-p_2.*p_13.*t15.*t30-p_4.*p_15.*t12.*t29-p_4.*p_17.*t15.*t24-p_2.*p_13.*t14.*t32-p_2.*p_13.*t16.*t30-p_2.*p_18.*t12.*t29-p_3.*p_14.*t12.*t32-p_3.*p_17.*t12.*t29-p_4.*p_15.*t13.*t29-p_2.*p_18.*t13.*t29-p_3.*p_14.*t13.*t32-p_3.*p_14.*t15.*t30-p_3.*p_17.*t13.*t29-p_4.*p_17.*t12.*t29-p_3.*p_14.*t14.*t32-p_3.*p_14.*t16.*t30-p_4.*p_17.*t13.*t29-p_2.*p_13.*t17.*t32-p_4.*p_15.*t17.*t29-p_4.*p_19.*t12.*t30-p_2.*p_18.*t17.*t29-p_3.*p_14.*t17.*t32-p_3.*p_17.*t17.*t29-p_4.*p_17.*t17.*t29-p_4.*p_19.*t12.*t32;
et3 = -p_4.*p_19.*t13.*t32-p_4.*p_19.*t15.*t30-p_4.*p_19.*t14.*t32-p_4.*p_19.*t16.*t30-p_4.*p_19.*t17.*t32-p_1.*p_12.*q_dot_2.*q_dot_5.*t24.*2.0-p_0.*p_16.*q_dot_2.*q_dot_7.*t20.*(3.0./5.0)-p_1.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0-p_1.*p_12.*q_dot_2.*q_dot_3.*t29.*2.0-p_2.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0-p_3.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0-p_4.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0-p_4.*p_15.*q_dot_2.*q_dot_5.*t24.*2.0-p_1.*p_12.*q_dot_2.*q_dot_7.*t29.*2.0-p_2.*p_18.*q_dot_2.*q_dot_5.*t24.*2.0-p_3.*p_17.*q_dot_2.*q_dot_5.*t24.*2.0-p_1.*p_12.*q_dot_3.*q_dot_7.*t29.*2.0-p_2.*p_13.*q_dot_2.*q_dot_3.*t32.*2.0-p_2.*p_13.*q_dot_2.*q_dot_5.*t30.*2.0-p_4.*p_17.*q_dot_2.*q_dot_5.*t24.*2.0-p_2.*p_13.*q_dot_2.*q_dot_4.*t32.*2.0-p_2.*p_13.*q_dot_2.*q_dot_6.*t30.*2.0-p_4.*p_15.*q_dot_2.*q_dot_3.*t29.*2.0-p_2.*p_13.*q_dot_3.*q_dot_4.*t32.*2.0-p_2.*p_18.*q_dot_2.*q_dot_3.*t29.*2.0;
et4 = p_3.*p_14.*q_dot_2.*q_dot_3.*t32.*-2.0-p_3.*p_14.*q_dot_2.*q_dot_5.*t30.*2.0-p_3.*p_17.*q_dot_2.*q_dot_3.*t29.*2.0-p_3.*p_14.*q_dot_2.*q_dot_4.*t32.*2.0-p_3.*p_14.*q_dot_2.*q_dot_6.*t30.*2.0-p_4.*p_17.*q_dot_2.*q_dot_3.*t29.*2.0-p_2.*p_13.*q_dot_2.*q_dot_7.*t32.*2.0-p_2.*p_13.*q_dot_5.*q_dot_6.*t30.*2.0-p_3.*p_14.*q_dot_3.*q_dot_4.*t32.*2.0-p_2.*p_13.*q_dot_3.*q_dot_7.*t32.*2.0-p_4.*p_15.*q_dot_2.*q_dot_7.*t29.*2.0-p_2.*p_13.*q_dot_4.*q_dot_7.*t32.*2.0-p_2.*p_18.*q_dot_2.*q_dot_7.*t29.*2.0-p_3.*p_14.*q_dot_2.*q_dot_7.*t32.*2.0-p_3.*p_14.*q_dot_5.*q_dot_6.*t30.*2.0-p_3.*p_17.*q_dot_2.*q_dot_7.*t29.*2.0-p_4.*p_15.*q_dot_3.*q_dot_7.*t29.*2.0-p_2.*p_18.*q_dot_3.*q_dot_7.*t29.*2.0-p_3.*p_14.*q_dot_3.*q_dot_7.*t32.*2.0-p_3.*p_17.*q_dot_3.*q_dot_7.*t29.*2.0-p_4.*p_17.*q_dot_2.*q_dot_7.*t29.*2.0-p_3.*p_14.*q_dot_4.*q_dot_7.*t32.*2.0-p_4.*p_17.*q_dot_3.*q_dot_7.*t29.*2.0-p_4.*p_19.*q_dot_2.*q_dot_3.*t32.*2.0-p_4.*p_19.*q_dot_2.*q_dot_5.*t30.*2.0;
et5 = p_4.*p_19.*q_dot_2.*q_dot_4.*t32.*-2.0-p_4.*p_19.*q_dot_2.*q_dot_6.*t30.*2.0-p_4.*p_19.*q_dot_3.*q_dot_4.*t32.*2.0-p_4.*p_19.*q_dot_2.*q_dot_7.*t32.*2.0-p_4.*p_19.*q_dot_5.*q_dot_6.*t30.*2.0-p_4.*p_19.*q_dot_3.*q_dot_7.*t32.*2.0-p_4.*p_19.*q_dot_4.*q_dot_7.*t32.*2.0;
et6 = p_0.*p_16.*t5.*t12.*(3.0./1.0e+1)+p_1.*p_16.*t5.*t12+p_2.*p_16.*t5.*t12+p_3.*p_16.*t5.*t12+p_4.*p_16.*t5.*t12+p_1.*p_12.*t12.*t18+p_1.*p_12.*t15.*t18+p_4.*p_15.*t12.*t18+p_2.*p_18.*t12.*t18+p_3.*p_17.*t12.*t18+p_1.*p_12.*t12.*t26+p_4.*p_17.*t12.*t18+p_1.*p_12.*t13.*t26+p_4.*p_15.*t15.*t18-p_0.*p_16.*t12.*t25.*(3.0./1.0e+1)+p_2.*p_18.*t15.*t18+p_3.*p_17.*t15.*t18-p_1.*p_16.*t12.*t25+p_2.*p_13.*t12.*t27+p_4.*p_17.*t15.*t18-p_2.*p_16.*t12.*t25+p_1.*p_12.*t17.*t26+p_3.*p_14.*t12.*t27-p_3.*p_16.*t12.*t25+p_2.*p_13.*t15.*t27+p_4.*p_15.*t12.*t26-p_4.*p_16.*t12.*t25-p_0.*p_16.*t17.*t25.*(3.0./1.0e+1)+p_2.*p_13.*t12.*t31+p_2.*p_13.*t16.*t27+p_2.*p_18.*t12.*t26+p_3.*p_17.*t12.*t26+p_4.*p_15.*t13.*t26-p_1.*p_16.*t17.*t25+p_2.*p_13.*t13.*t31;
et7 = p_2.*p_18.*t13.*t26+p_3.*p_14.*t15.*t27+p_3.*p_17.*t13.*t26+p_4.*p_17.*t12.*t26+p_2.*p_13.*t14.*t31-p_2.*p_16.*t17.*t25+p_3.*p_14.*t12.*t31+p_3.*p_14.*t16.*t27+p_4.*p_17.*t13.*t26+p_3.*p_14.*t13.*t31-p_3.*p_16.*t17.*t25+p_3.*p_14.*t14.*t31+p_4.*p_15.*t17.*t26-p_4.*p_16.*t17.*t25+p_4.*p_19.*t12.*t27+p_2.*p_13.*t17.*t31+p_2.*p_18.*t17.*t26+p_3.*p_17.*t17.*t26+p_4.*p_17.*t17.*t26+p_3.*p_14.*t17.*t31+p_4.*p_19.*t15.*t27+p_4.*p_19.*t12.*t31+p_4.*p_19.*t16.*t27+p_4.*p_19.*t13.*t31+p_4.*p_19.*t14.*t31+p_4.*p_19.*t17.*t31+p_1.*p_12.*q_dot_2.*q_dot_5.*t18.*2.0+p_1.*p_12.*q_dot_2.*q_dot_3.*t26.*2.0+p_4.*p_15.*q_dot_2.*q_dot_5.*t18.*2.0+p_2.*p_18.*q_dot_2.*q_dot_5.*t18.*2.0+p_3.*p_17.*q_dot_2.*q_dot_5.*t18.*2.0+p_4.*p_17.*q_dot_2.*q_dot_5.*t18.*2.0+p_1.*p_12.*q_dot_2.*q_dot_7.*t26.*2.0+p_1.*p_12.*q_dot_3.*q_dot_7.*t26.*2.0+p_2.*p_13.*q_dot_2.*q_dot_5.*t27.*2.0;
et8 = p_0.*p_16.*q_dot_2.*q_dot_7.*t25.*(-3.0./5.0)+p_2.*p_13.*q_dot_2.*q_dot_6.*t27.*2.0+p_4.*p_15.*q_dot_2.*q_dot_3.*t26.*2.0-p_1.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0+p_2.*p_13.*q_dot_2.*q_dot_3.*t31.*2.0+p_2.*p_18.*q_dot_2.*q_dot_3.*t26.*2.0+p_3.*p_14.*q_dot_2.*q_dot_5.*t27.*2.0+p_3.*p_17.*q_dot_2.*q_dot_3.*t26.*2.0+p_2.*p_13.*q_dot_2.*q_dot_4.*t31.*2.0-p_2.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0+p_3.*p_14.*q_dot_2.*q_dot_6.*t27.*2.0+p_4.*p_17.*q_dot_2.*q_dot_3.*t26.*2.0+p_2.*p_13.*q_dot_3.*q_dot_4.*t31.*2.0+p_2.*p_13.*q_dot_5.*q_dot_6.*t27.*2.0+p_3.*p_14.*q_dot_2.*q_dot_3.*t31.*2.0-p_3.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0+p_3.*p_14.*q_dot_2.*q_dot_4.*t31.*2.0+p_4.*p_15.*q_dot_2.*q_dot_7.*t26.*2.0-p_4.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0+p_2.*p_13.*q_dot_2.*q_dot_7.*t31.*2.0+p_2.*p_18.*q_dot_2.*q_dot_7.*t26.*2.0+p_3.*p_14.*q_dot_3.*q_dot_4.*t31.*2.0+p_3.*p_14.*q_dot_5.*q_dot_6.*t27.*2.0+p_3.*p_17.*q_dot_2.*q_dot_7.*t26.*2.0+p_4.*p_15.*q_dot_3.*q_dot_7.*t26.*2.0+p_2.*p_13.*q_dot_3.*q_dot_7.*t31.*2.0+p_2.*p_18.*q_dot_3.*q_dot_7.*t26.*2.0;
et9 = p_3.*p_17.*q_dot_3.*q_dot_7.*t26.*2.0+p_4.*p_17.*q_dot_2.*q_dot_7.*t26.*2.0+p_2.*p_13.*q_dot_4.*q_dot_7.*t31.*2.0+p_3.*p_14.*q_dot_2.*q_dot_7.*t31.*2.0+p_4.*p_17.*q_dot_3.*q_dot_7.*t26.*2.0+p_4.*p_19.*q_dot_2.*q_dot_5.*t27.*2.0+p_3.*p_14.*q_dot_3.*q_dot_7.*t31.*2.0+p_4.*p_19.*q_dot_2.*q_dot_6.*t27.*2.0+p_3.*p_14.*q_dot_4.*q_dot_7.*t31.*2.0+p_4.*p_19.*q_dot_2.*q_dot_3.*t31.*2.0+p_4.*p_19.*q_dot_2.*q_dot_4.*t31.*2.0+p_4.*p_19.*q_dot_3.*q_dot_4.*t31.*2.0+p_4.*p_19.*q_dot_5.*q_dot_6.*t27.*2.0+p_4.*p_19.*q_dot_2.*q_dot_7.*t31.*2.0+p_4.*p_19.*q_dot_3.*q_dot_7.*t31.*2.0+p_4.*p_19.*q_dot_4.*q_dot_7.*t31.*2.0;
et10 = t33+t34+t35+t36+t37+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t76+t77+t78+t79+t80+t81+t82+t83+t84+t85+t86+t87+t88+t89+t90+t91+t92+t93+t94+t95+t97+t98+t100+t102+t107+t108+t111+t112+t113+t114+t115+t117+t118+t119+t121+t122+t123+t124+t125+t127+t128+t129+t130+t131+t132+t133+t134+t135+t136-p_1.*p_12.*p_16.*t4.*t15-p_4.*p_15.*p_16.*t4.*t15-p_2.*p_16.*p_18.*t4.*t15-p_3.*p_16.*p_17.*t4.*t15-p_4.*p_16.*p_17.*t4.*t15-p_2.*p_13.*p_16.*t15.*t21-p_2.*p_13.*p_16.*t16.*t21-p_3.*p_14.*p_16.*t15.*t21-p_3.*p_14.*p_16.*t16.*t21-p_4.*p_16.*p_19.*t15.*t21-p_4.*p_16.*p_19.*t16.*t21-p_1.*p_12.*p_16.*q_dot_2.*q_dot_5.*t4.*2.0-p_4.*p_15.*p_16.*q_dot_2.*q_dot_5.*t4.*2.0-p_2.*p_16.*p_18.*q_dot_2.*q_dot_5.*t4.*2.0-p_3.*p_16.*p_17.*q_dot_2.*q_dot_5.*t4.*2.0-p_4.*p_16.*p_17.*q_dot_2.*q_dot_5.*t4.*2.0;
et11 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_5.*t21.*-2.0-p_2.*p_13.*p_16.*q_dot_2.*q_dot_6.*t21.*2.0-p_3.*p_14.*p_16.*q_dot_2.*q_dot_5.*t21.*2.0-p_3.*p_14.*p_16.*q_dot_2.*q_dot_6.*t21.*2.0-p_2.*p_13.*p_16.*q_dot_5.*q_dot_6.*t21.*2.0-p_3.*p_14.*p_16.*q_dot_5.*q_dot_6.*t21.*2.0-p_4.*p_16.*p_19.*q_dot_2.*q_dot_5.*t21.*2.0-p_4.*p_16.*p_19.*q_dot_2.*q_dot_6.*t21.*2.0-p_4.*p_16.*p_19.*q_dot_5.*q_dot_6.*t21.*2.0;
mt1 = [et1+et2+et3+et4+et5;et6+et7+et8+et9;et10+et11;t76+t77+t78+t79+t81+t83+t84+t85+t87+t89+t91+t94+t129+t130+t133+t134+t137+t138+t139+t140+t141+t142+t143+t144+t145-p_1.*p_12.*p_16.*t3.*t12-p_1.*p_12.*p_16.*t3.*t17-p_4.*p_15.*p_16.*t3.*t12-p_2.*p_16.*p_18.*t3.*t12-p_3.*p_16.*p_17.*t3.*t12-p_4.*p_16.*p_17.*t3.*t12-p_4.*p_15.*p_16.*t3.*t17-p_2.*p_16.*p_18.*t3.*t17-p_3.*p_16.*p_17.*t3.*t17-p_4.*p_16.*p_17.*t3.*t17-p_1.*p_12.*p_16.*q_dot_2.*q_dot_7.*t3.*2.0-p_4.*p_15.*p_16.*q_dot_2.*q_dot_7.*t3.*2.0-p_2.*p_16.*p_18.*q_dot_2.*q_dot_7.*t3.*2.0-p_3.*p_16.*p_17.*q_dot_2.*q_dot_7.*t3.*2.0-p_4.*p_16.*p_17.*q_dot_2.*q_dot_7.*t3.*2.0];
mt2 = [t137+t138+t139+t140+t141+t142+t143+t144+t145+p_2.*p_13.*p_18.*t6.*t12+p_2.*p_13.*p_18.*t6.*t13+p_3.*p_14.*p_17.*t6.*t12+p_3.*p_14.*p_17.*t6.*t13+p_2.*p_13.*p_18.*t6.*t17+p_4.*p_15.*p_19.*t6.*t12+p_3.*p_14.*p_17.*t6.*t17+p_4.*p_15.*p_19.*t6.*t13+p_4.*p_17.*p_19.*t6.*t12+p_4.*p_17.*p_19.*t6.*t13+p_4.*p_15.*p_19.*t6.*t17+p_4.*p_17.*p_19.*t6.*t17+p_2.*p_13.*p_18.*q_dot_2.*q_dot_3.*t6.*2.0+p_3.*p_14.*p_17.*q_dot_2.*q_dot_3.*t6.*2.0+p_2.*p_13.*p_18.*q_dot_2.*q_dot_7.*t6.*2.0+p_2.*p_13.*p_18.*q_dot_3.*q_dot_7.*t6.*2.0+p_3.*p_14.*p_17.*q_dot_2.*q_dot_7.*t6.*2.0+p_4.*p_15.*p_19.*q_dot_2.*q_dot_3.*t6.*2.0+p_3.*p_14.*p_17.*q_dot_3.*q_dot_7.*t6.*2.0+p_4.*p_17.*p_19.*q_dot_2.*q_dot_3.*t6.*2.0+p_4.*p_15.*p_19.*q_dot_2.*q_dot_7.*t6.*2.0+p_4.*p_15.*p_19.*q_dot_3.*q_dot_7.*t6.*2.0+p_4.*p_17.*p_19.*q_dot_2.*q_dot_7.*t6.*2.0+p_4.*p_17.*p_19.*q_dot_3.*q_dot_7.*t6.*2.0];
mt3 = [t80+t82+t86+t88+t90+t92+t93+t95+t101+t104+t109+t131+t132+t135+t136+p_1.*p_12.*p_16.*t4.*t12+p_4.*p_15.*p_16.*t4.*t12+p_2.*p_16.*p_18.*t4.*t12+p_3.*p_16.*p_17.*t4.*t12+p_4.*p_16.*p_17.*t4.*t12;t101+t104+t109+p_2.*p_13.*p_18.*t7.*t12+p_3.*p_14.*p_17.*t7.*t12+p_2.*p_13.*p_18.*t7.*t15+p_3.*p_14.*p_17.*t7.*t15+p_4.*p_15.*p_19.*t7.*t12+p_4.*p_17.*p_19.*t7.*t12+p_4.*p_15.*p_19.*t7.*t15+p_4.*p_17.*p_19.*t7.*t15+p_2.*p_13.*p_18.*q_dot_2.*q_dot_5.*t7.*2.0+p_3.*p_14.*p_17.*q_dot_2.*q_dot_5.*t7.*2.0+p_4.*p_15.*p_19.*q_dot_2.*q_dot_5.*t7.*2.0+p_4.*p_17.*p_19.*q_dot_2.*q_dot_5.*t7.*2.0;t33+t34+t35+t36+t37+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t76+t77+t78+t79+t81+t83+t84+t85+t87+t89+t91+t94+t97+t98+t100+t102+t107+t108+t111+t112+t113+t114+t115+t117+t118+t119+t121+t122+t123+t124+t125+t127+t128+t129+t130+t133+t134];
C = [mt1;mt2;mt3];
end

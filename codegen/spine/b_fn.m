function b = b_fn(in1,in2,in3)
%B_FN
%    B = B_FN(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:03

p22 = in3(23,:);
p23 = in3(24,:);
p_0 = in3(1,:);
p_1 = in3(2,:);
p_2 = in3(3,:);
p_3 = in3(4,:);
p_4 = in3(5,:);
p_12 = in3(13,:);
p_13 = in3(14,:);
p_14 = in3(15,:);
p_15 = in3(16,:);
p_16 = in3(17,:);
p_17 = in3(18,:);
p_18 = in3(19,:);
p_19 = in3(20,:);
p_21 = in3(22,:);
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
u_0 = in2(1,:);
u_1 = in2(2,:);
u_2 = in2(3,:);
u_3 = in2(4,:);
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
t42 = p_1.*p_12.*p_16.*t3.*t13;
t43 = p_4.*p_15.*p_16.*t3.*t13;
t44 = p_2.*p_16.*p_18.*t3.*t13;
t45 = p_3.*p_16.*p_17.*t3.*t13;
t46 = p_4.*p_16.*p_17.*t3.*t13;
t47 = p_2.*p_13.*p_18.*t6.*t14;
t48 = p_3.*p_14.*p_17.*t6.*t14;
t49 = p_2.*p_13.*p_18.*t7.*t16;
t50 = p_3.*p_14.*p_17.*t7.*t16;
t51 = p_4.*p_15.*p_19.*t6.*t14;
t52 = p_4.*p_17.*p_19.*t6.*t14;
t53 = p_4.*p_15.*p_19.*t7.*t16;
t54 = p_4.*p_17.*p_19.*t7.*t16;
t55 = p_1.*p_12.*p_16.*q_dot_2.*q_dot_3.*t3.*2.0;
t56 = p_1.*p_12.*p_16.*q_dot_3.*q_dot_7.*t3.*2.0;
t57 = p_4.*p_15.*p_16.*q_dot_2.*q_dot_3.*t3.*2.0;
t58 = p_2.*p_16.*p_18.*q_dot_2.*q_dot_3.*t3.*2.0;
t59 = p_3.*p_16.*p_17.*q_dot_2.*q_dot_3.*t3.*2.0;
t60 = p_4.*p_16.*p_17.*q_dot_2.*q_dot_3.*t3.*2.0;
t61 = p_4.*p_15.*p_16.*q_dot_3.*q_dot_7.*t3.*2.0;
t62 = p_2.*p_16.*p_18.*q_dot_3.*q_dot_7.*t3.*2.0;
t63 = p_3.*p_16.*p_17.*q_dot_3.*q_dot_7.*t3.*2.0;
t64 = p_4.*p_16.*p_17.*q_dot_3.*q_dot_7.*t3.*2.0;
t65 = p_2.*p_13.*p_18.*q_dot_2.*q_dot_4.*t6.*2.0;
t66 = p_2.*p_13.*p_18.*q_dot_3.*q_dot_4.*t6.*2.0;
t67 = p_3.*p_14.*p_17.*q_dot_2.*q_dot_4.*t6.*2.0;
t68 = p_3.*p_14.*p_17.*q_dot_3.*q_dot_4.*t6.*2.0;
t69 = p_2.*p_13.*p_18.*q_dot_2.*q_dot_6.*t7.*2.0;
t70 = p_2.*p_13.*p_18.*q_dot_4.*q_dot_7.*t6.*2.0;
t71 = p_3.*p_14.*p_17.*q_dot_2.*q_dot_6.*t7.*2.0;
t72 = p_4.*p_15.*p_19.*q_dot_2.*q_dot_4.*t6.*2.0;
t73 = p_3.*p_14.*p_17.*q_dot_4.*q_dot_7.*t6.*2.0;
t74 = p_4.*p_15.*p_19.*q_dot_3.*q_dot_4.*t6.*2.0;
t75 = p_2.*p_13.*p_18.*q_dot_5.*q_dot_6.*t7.*2.0;
t76 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_4.*t6.*2.0;
t77 = p_3.*p_14.*p_17.*q_dot_5.*q_dot_6.*t7.*2.0;
t78 = p_4.*p_17.*p_19.*q_dot_3.*q_dot_4.*t6.*2.0;
t79 = p_4.*p_15.*p_19.*q_dot_2.*q_dot_6.*t7.*2.0;
t80 = p_4.*p_15.*p_19.*q_dot_4.*q_dot_7.*t6.*2.0;
t81 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_6.*t7.*2.0;
t82 = p_4.*p_15.*p_19.*q_dot_5.*q_dot_6.*t7.*2.0;
t83 = p_4.*p_17.*p_19.*q_dot_4.*q_dot_7.*t6.*2.0;
t84 = p_4.*p_17.*p_19.*q_dot_5.*q_dot_6.*t7.*2.0;
t26 = cos(t22);
t27 = cos(t23);
t29 = sin(t22);
t30 = sin(t23);
t31 = cos(t28);
t32 = sin(t28);
t33 = p_1.*p_16.*p_21.*t20;
t34 = p_2.*p_16.*p_21.*t20;
t35 = p_3.*p_16.*p_21.*t20;
t36 = p_4.*p_16.*p_21.*t20;
t37 = p_1.*p_12.*p_21.*t24;
t38 = p_4.*p_15.*p_21.*t24;
t39 = p_2.*p_18.*p_21.*t24;
t40 = p_3.*p_17.*p_21.*t24;
t41 = p_4.*p_17.*p_21.*t24;
t105 = -t55;
t106 = -t56;
t107 = -t57;
t108 = -t58;
t109 = -t59;
t110 = -t60;
t111 = -t61;
t112 = -t62;
t113 = -t63;
t114 = -t64;
t115 = p_2.*p_13.*p_16.*t12.*t19;
t116 = p_2.*p_13.*p_16.*t13.*t19;
t117 = p_2.*p_13.*p_16.*t14.*t19;
t118 = p_3.*p_14.*p_16.*t12.*t19;
t119 = p_3.*p_14.*p_16.*t13.*t19;
t120 = p_2.*p_13.*p_16.*t12.*t21;
t121 = p_3.*p_14.*p_16.*t14.*t19;
t122 = p_2.*p_13.*p_16.*t17.*t19;
t123 = p_3.*p_14.*p_16.*t12.*t21;
t124 = p_3.*p_14.*p_16.*t17.*t19;
t125 = p_4.*p_16.*p_19.*t12.*t19;
t126 = p_4.*p_16.*p_19.*t13.*t19;
t127 = p_4.*p_16.*p_19.*t14.*t19;
t128 = p_4.*p_16.*p_19.*t12.*t21;
t129 = p_4.*p_16.*p_19.*t17.*t19;
t130 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_3.*t19.*2.0;
t131 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_4.*t19.*2.0;
t132 = p_2.*p_13.*p_16.*q_dot_3.*q_dot_4.*t19.*2.0;
t133 = p_3.*p_14.*p_16.*q_dot_2.*q_dot_3.*t19.*2.0;
t134 = p_3.*p_14.*p_16.*q_dot_2.*q_dot_4.*t19.*2.0;
t135 = p_2.*p_13.*p_16.*q_dot_2.*q_dot_7.*t19.*2.0;
t136 = p_3.*p_14.*p_16.*q_dot_3.*q_dot_4.*t19.*2.0;
t137 = p_2.*p_13.*p_16.*q_dot_3.*q_dot_7.*t19.*2.0;
t138 = p_2.*p_13.*p_16.*q_dot_4.*q_dot_7.*t19.*2.0;
t139 = p_3.*p_14.*p_16.*q_dot_2.*q_dot_7.*t19.*2.0;
t140 = p_3.*p_14.*p_16.*q_dot_3.*q_dot_7.*t19.*2.0;
t141 = p_3.*p_14.*p_16.*q_dot_4.*q_dot_7.*t19.*2.0;
t142 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_3.*t19.*2.0;
t143 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_4.*t19.*2.0;
t144 = p_4.*p_16.*p_19.*q_dot_3.*q_dot_4.*t19.*2.0;
t145 = p_4.*p_16.*p_19.*q_dot_2.*q_dot_7.*t19.*2.0;
t146 = p_4.*p_16.*p_19.*q_dot_3.*q_dot_7.*t19.*2.0;
t147 = p_4.*p_16.*p_19.*q_dot_4.*q_dot_7.*t19.*2.0;
t148 = -t42;
t149 = -t43;
t150 = -t44;
t151 = -t45;
t152 = -t46;
t179 = p_0.*p_16.*p_21.*t20.*(3.0./1.0e+1);
t85 = p_1.*p_12.*p_21.*t29;
t86 = p_2.*p_13.*p_21.*t30;
t87 = p_3.*p_14.*p_21.*t30;
t88 = p_4.*p_15.*p_21.*t29;
t89 = p_2.*p_18.*p_21.*t29;
t90 = p_3.*p_17.*p_21.*t29;
t91 = p_4.*p_17.*p_21.*t29;
t92 = p_4.*p_19.*p_21.*t30;
t93 = p_2.*p_13.*p_21.*t32;
t94 = p_3.*p_14.*p_21.*t32;
t95 = p_4.*p_19.*p_21.*t32;
t96 = -t33;
t97 = -t34;
t98 = -t35;
t99 = -t36;
t100 = -t37;
t101 = -t38;
t102 = -t39;
t103 = -t40;
t104 = -t41;
t161 = -t130;
t162 = -t131;
t163 = -t132;
t164 = -t133;
t165 = -t134;
t166 = -t136;
t167 = -t137;
t168 = -t138;
t169 = -t140;
t170 = -t141;
t171 = -t142;
t172 = -t143;
t173 = -t144;
t174 = -t146;
t175 = -t147;
t180 = -t116;
t181 = -t117;
t182 = -t119;
t183 = -t120;
t184 = -t121;
t185 = -t123;
t186 = -t126;
t187 = -t127;
t188 = -t128;
t189 = -t179;
t153 = -t85;
t154 = -t86;
t155 = -t87;
t156 = -t88;
t157 = -t89;
t158 = -t90;
t159 = -t91;
t160 = -t92;
t176 = -t93;
t177 = -t94;
t178 = -t95;
et1 = p_0.*p_16.*t2.*t12.*(-3.0./1.0e+1)-p_1.*p_16.*t2.*t12-p_2.*p_16.*t2.*t12-p_3.*p_16.*t2.*t12-p_4.*p_16.*t2.*t12+p_0.*p_16.*t12.*t20.*(3.0./1.0e+1)+p_1.*p_12.*t12.*t24+p_1.*p_16.*t12.*t20+p_2.*p_16.*t12.*t20+p_3.*p_16.*t12.*t20+p_1.*p_12.*t15.*t24+p_4.*p_16.*t12.*t20+p_0.*p_16.*t17.*t20.*(3.0./1.0e+1)+p_1.*p_12.*t12.*t29+p_1.*p_16.*t17.*t20+p_1.*p_12.*t13.*t29+p_2.*p_16.*t17.*t20+p_4.*p_15.*t12.*t24+p_2.*p_18.*t12.*t24+p_3.*p_16.*t17.*t20+p_3.*p_17.*t12.*t24+p_2.*p_13.*t12.*t30+p_4.*p_16.*t17.*t20+p_4.*p_17.*t12.*t24+p_4.*p_15.*t15.*t24+p_1.*p_12.*t17.*t29+p_2.*p_13.*t12.*t32+p_2.*p_18.*t15.*t24+p_3.*p_14.*t12.*t30+p_3.*p_17.*t15.*t24+p_2.*p_13.*t13.*t32+p_2.*p_13.*t15.*t30+p_4.*p_15.*t12.*t29+p_4.*p_17.*t15.*t24+p_2.*p_13.*t14.*t32;
et2 = p_2.*p_13.*t16.*t30+p_2.*p_18.*t12.*t29+p_3.*p_14.*t12.*t32+p_3.*p_17.*t12.*t29+p_4.*p_15.*t13.*t29+p_2.*p_18.*t13.*t29+p_3.*p_14.*t13.*t32+p_3.*p_14.*t15.*t30+p_3.*p_17.*t13.*t29+p_4.*p_17.*t12.*t29+p_3.*p_14.*t14.*t32+p_3.*p_14.*t16.*t30+p_4.*p_17.*t13.*t29+p_2.*p_13.*t17.*t32+p_4.*p_15.*t17.*t29+p_4.*p_19.*t12.*t30+p_2.*p_18.*t17.*t29+p_3.*p_14.*t17.*t32+p_3.*p_17.*t17.*t29+p_4.*p_17.*t17.*t29+p_4.*p_19.*t12.*t32+p_4.*p_19.*t13.*t32+p_4.*p_19.*t15.*t30+p_4.*p_19.*t14.*t32+p_4.*p_19.*t16.*t30+p_4.*p_19.*t17.*t32+p_1.*p_12.*q_dot_2.*q_dot_5.*t24.*2.0+p_0.*p_16.*q_dot_2.*q_dot_7.*t20.*(3.0./5.0)+p_1.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0+p_1.*p_12.*q_dot_2.*q_dot_3.*t29.*2.0+p_2.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0+p_3.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0+p_4.*p_16.*q_dot_2.*q_dot_7.*t20.*2.0+p_4.*p_15.*q_dot_2.*q_dot_5.*t24.*2.0+p_1.*p_12.*q_dot_2.*q_dot_7.*t29.*2.0;
et3 = p_2.*p_18.*q_dot_2.*q_dot_5.*t24.*2.0+p_3.*p_17.*q_dot_2.*q_dot_5.*t24.*2.0+p_1.*p_12.*q_dot_3.*q_dot_7.*t29.*2.0+p_2.*p_13.*q_dot_2.*q_dot_3.*t32.*2.0+p_2.*p_13.*q_dot_2.*q_dot_5.*t30.*2.0+p_4.*p_17.*q_dot_2.*q_dot_5.*t24.*2.0+p_2.*p_13.*q_dot_2.*q_dot_4.*t32.*2.0+p_2.*p_13.*q_dot_2.*q_dot_6.*t30.*2.0+p_4.*p_15.*q_dot_2.*q_dot_3.*t29.*2.0+p_2.*p_13.*q_dot_3.*q_dot_4.*t32.*2.0+p_2.*p_18.*q_dot_2.*q_dot_3.*t29.*2.0+p_3.*p_14.*q_dot_2.*q_dot_3.*t32.*2.0+p_3.*p_14.*q_dot_2.*q_dot_5.*t30.*2.0+p_3.*p_17.*q_dot_2.*q_dot_3.*t29.*2.0+p_3.*p_14.*q_dot_2.*q_dot_4.*t32.*2.0+p_3.*p_14.*q_dot_2.*q_dot_6.*t30.*2.0+p_4.*p_17.*q_dot_2.*q_dot_3.*t29.*2.0+p_2.*p_13.*q_dot_2.*q_dot_7.*t32.*2.0+p_2.*p_13.*q_dot_5.*q_dot_6.*t30.*2.0+p_3.*p_14.*q_dot_3.*q_dot_4.*t32.*2.0+p_2.*p_13.*q_dot_3.*q_dot_7.*t32.*2.0+p_4.*p_15.*q_dot_2.*q_dot_7.*t29.*2.0+p_2.*p_13.*q_dot_4.*q_dot_7.*t32.*2.0+p_2.*p_18.*q_dot_2.*q_dot_7.*t29.*2.0+p_3.*p_14.*q_dot_2.*q_dot_7.*t32.*2.0+p_3.*p_14.*q_dot_5.*q_dot_6.*t30.*2.0+p_3.*p_17.*q_dot_2.*q_dot_7.*t29.*2.0+p_4.*p_15.*q_dot_3.*q_dot_7.*t29.*2.0;
et4 = p_2.*p_18.*q_dot_3.*q_dot_7.*t29.*2.0+p_3.*p_14.*q_dot_3.*q_dot_7.*t32.*2.0+p_3.*p_17.*q_dot_3.*q_dot_7.*t29.*2.0+p_4.*p_17.*q_dot_2.*q_dot_7.*t29.*2.0+p_3.*p_14.*q_dot_4.*q_dot_7.*t32.*2.0+p_4.*p_17.*q_dot_3.*q_dot_7.*t29.*2.0+p_4.*p_19.*q_dot_2.*q_dot_3.*t32.*2.0+p_4.*p_19.*q_dot_2.*q_dot_5.*t30.*2.0+p_4.*p_19.*q_dot_2.*q_dot_4.*t32.*2.0+p_4.*p_19.*q_dot_2.*q_dot_6.*t30.*2.0+p_4.*p_19.*q_dot_3.*q_dot_4.*t32.*2.0+p_4.*p_19.*q_dot_2.*q_dot_7.*t32.*2.0+p_4.*p_19.*q_dot_5.*q_dot_6.*t30.*2.0+p_4.*p_19.*q_dot_3.*q_dot_7.*t32.*2.0+p_4.*p_19.*q_dot_4.*q_dot_7.*t32.*2.0;
et5 = -p_0.*p_21-p_1.*p_21.*2.0-p_2.*p_21.*2.0-p_3.*p_21.*2.0-p_4.*p_21.*2.0-p_0.*p_16.*t5.*t12.*(3.0./1.0e+1)-p_1.*p_16.*t5.*t12-p_2.*p_16.*t5.*t12-p_3.*p_16.*t5.*t12-p_4.*p_16.*t5.*t12-p_1.*p_12.*t12.*t18-p_1.*p_12.*t15.*t18-p_4.*p_15.*t12.*t18-p_2.*p_18.*t12.*t18-p_3.*p_17.*t12.*t18-p_1.*p_12.*t12.*t26-p_4.*p_17.*t12.*t18-p_1.*p_12.*t13.*t26-p_4.*p_15.*t15.*t18+p_0.*p_16.*t12.*t25.*(3.0./1.0e+1)-p_2.*p_18.*t15.*t18-p_3.*p_17.*t15.*t18+p_1.*p_16.*t12.*t25-p_2.*p_13.*t12.*t27-p_4.*p_17.*t15.*t18+p_2.*p_16.*t12.*t25-p_1.*p_12.*t17.*t26-p_3.*p_14.*t12.*t27+p_3.*p_16.*t12.*t25-p_2.*p_13.*t15.*t27;
et6 = -p_4.*p_15.*t12.*t26+p_4.*p_16.*t12.*t25+p_0.*p_16.*t17.*t25.*(3.0./1.0e+1)-p_2.*p_13.*t12.*t31-p_2.*p_13.*t16.*t27-p_2.*p_18.*t12.*t26-p_3.*p_17.*t12.*t26-p_4.*p_15.*t13.*t26+p_1.*p_16.*t17.*t25-p_2.*p_13.*t13.*t31-p_2.*p_18.*t13.*t26-p_3.*p_14.*t15.*t27-p_3.*p_17.*t13.*t26-p_4.*p_17.*t12.*t26-p_2.*p_13.*t14.*t31+p_2.*p_16.*t17.*t25-p_3.*p_14.*t12.*t31-p_3.*p_14.*t16.*t27-p_4.*p_17.*t13.*t26-p_3.*p_14.*t13.*t31+p_3.*p_16.*t17.*t25-p_3.*p_14.*t14.*t31-p_4.*p_15.*t17.*t26+p_4.*p_16.*t17.*t25-p_4.*p_19.*t12.*t27-p_2.*p_13.*t17.*t31-p_2.*p_18.*t17.*t26-p_3.*p_17.*t17.*t26-p_4.*p_17.*t17.*t26;
et7 = -p_3.*p_14.*t17.*t31-p_4.*p_19.*t15.*t27-p_4.*p_19.*t12.*t31-p_4.*p_19.*t16.*t27-p_4.*p_19.*t13.*t31-p_4.*p_19.*t14.*t31-p_4.*p_19.*t17.*t31-p_1.*p_12.*q_dot_2.*q_dot_5.*t18.*2.0-p_1.*p_12.*q_dot_2.*q_dot_3.*t26.*2.0-p_4.*p_15.*q_dot_2.*q_dot_5.*t18.*2.0-p_2.*p_18.*q_dot_2.*q_dot_5.*t18.*2.0-p_3.*p_17.*q_dot_2.*q_dot_5.*t18.*2.0-p_4.*p_17.*q_dot_2.*q_dot_5.*t18.*2.0-p_1.*p_12.*q_dot_2.*q_dot_7.*t26.*2.0-p_1.*p_12.*q_dot_3.*q_dot_7.*t26.*2.0-p_2.*p_13.*q_dot_2.*q_dot_5.*t27.*2.0+p_0.*p_16.*q_dot_2.*q_dot_7.*t25.*(3.0./5.0)-p_2.*p_13.*q_dot_2.*q_dot_6.*t27.*2.0-p_4.*p_15.*q_dot_2.*q_dot_3.*t26.*2.0+p_1.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0-p_2.*p_13.*q_dot_2.*q_dot_3.*t31.*2.0-p_2.*p_18.*q_dot_2.*q_dot_3.*t26.*2.0-p_3.*p_14.*q_dot_2.*q_dot_5.*t27.*2.0-p_3.*p_17.*q_dot_2.*q_dot_3.*t26.*2.0-p_2.*p_13.*q_dot_2.*q_dot_4.*t31.*2.0+p_2.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0;
et8 = p_3.*p_14.*q_dot_2.*q_dot_6.*t27.*-2.0-p_4.*p_17.*q_dot_2.*q_dot_3.*t26.*2.0-p_2.*p_13.*q_dot_3.*q_dot_4.*t31.*2.0-p_2.*p_13.*q_dot_5.*q_dot_6.*t27.*2.0-p_3.*p_14.*q_dot_2.*q_dot_3.*t31.*2.0+p_3.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0-p_3.*p_14.*q_dot_2.*q_dot_4.*t31.*2.0-p_4.*p_15.*q_dot_2.*q_dot_7.*t26.*2.0+p_4.*p_16.*q_dot_2.*q_dot_7.*t25.*2.0-p_2.*p_13.*q_dot_2.*q_dot_7.*t31.*2.0-p_2.*p_18.*q_dot_2.*q_dot_7.*t26.*2.0-p_3.*p_14.*q_dot_3.*q_dot_4.*t31.*2.0-p_3.*p_14.*q_dot_5.*q_dot_6.*t27.*2.0-p_3.*p_17.*q_dot_2.*q_dot_7.*t26.*2.0-p_4.*p_15.*q_dot_3.*q_dot_7.*t26.*2.0-p_2.*p_13.*q_dot_3.*q_dot_7.*t31.*2.0-p_2.*p_18.*q_dot_3.*q_dot_7.*t26.*2.0-p_3.*p_17.*q_dot_3.*q_dot_7.*t26.*2.0-p_4.*p_17.*q_dot_2.*q_dot_7.*t26.*2.0-p_2.*p_13.*q_dot_4.*q_dot_7.*t31.*2.0-p_3.*p_14.*q_dot_2.*q_dot_7.*t31.*2.0-p_4.*p_17.*q_dot_3.*q_dot_7.*t26.*2.0-p_4.*p_19.*q_dot_2.*q_dot_5.*t27.*2.0-p_3.*p_14.*q_dot_3.*q_dot_7.*t31.*2.0-p_4.*p_19.*q_dot_2.*q_dot_6.*t27.*2.0;
et9 = p_3.*p_14.*q_dot_4.*q_dot_7.*t31.*-2.0-p_4.*p_19.*q_dot_2.*q_dot_3.*t31.*2.0-p_4.*p_19.*q_dot_2.*q_dot_4.*t31.*2.0-p_4.*p_19.*q_dot_3.*q_dot_4.*t31.*2.0-p_4.*p_19.*q_dot_5.*q_dot_6.*t27.*2.0-p_4.*p_19.*q_dot_2.*q_dot_7.*t31.*2.0-p_4.*p_19.*q_dot_3.*q_dot_7.*t31.*2.0-p_4.*p_19.*q_dot_4.*q_dot_7.*t31.*2.0;
et10 = t47+t48+t49+t50+t51+t52+t53+t54+t65+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78+t79+t80+t81+t82+t83+t84+t96+t97+t98+t99+t100+t101+t102+t103+t104+t105+t106+t107+t108+t109+t110+t111+t112+t113+t114+t148+t149+t150+t151+t152+t153+t154+t155+t156+t157+t158+t159+t160+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t173+t174+t175+t176+t177+t178+t180+t181+t182+t184+t186+t187+t189+p_0.*p_16.*p_21.*t2.*(3.0./1.0e+1)+p_1.*p_16.*p_21.*t2+p_2.*p_16.*p_21.*t2+p_3.*p_16.*p_21.*t2+p_4.*p_16.*p_21.*t2+p_1.*p_12.*p_16.*t4.*t15+p_4.*p_15.*p_16.*t4.*t15+p_2.*p_16.*p_18.*t4.*t15+p_3.*p_16.*p_17.*t4.*t15+p_4.*p_16.*p_17.*t4.*t15+p_2.*p_13.*p_16.*t15.*t21+p_2.*p_13.*p_16.*t16.*t21+p_3.*p_14.*p_16.*t15.*t21+p_3.*p_14.*p_16.*t16.*t21+p_4.*p_16.*p_19.*t15.*t21+p_4.*p_16.*p_19.*t16.*t21+p_1.*p_12.*p_16.*q_dot_2.*q_dot_5.*t4.*2.0+p_4.*p_15.*p_16.*q_dot_2.*q_dot_5.*t4.*2.0;
et11 = p_2.*p_16.*p_18.*q_dot_2.*q_dot_5.*t4.*2.0+p_3.*p_16.*p_17.*q_dot_2.*q_dot_5.*t4.*2.0+p_4.*p_16.*p_17.*q_dot_2.*q_dot_5.*t4.*2.0+p_2.*p_13.*p_16.*q_dot_2.*q_dot_5.*t21.*2.0+p_2.*p_13.*p_16.*q_dot_2.*q_dot_6.*t21.*2.0+p_3.*p_14.*p_16.*q_dot_2.*q_dot_5.*t21.*2.0+p_3.*p_14.*p_16.*q_dot_2.*q_dot_6.*t21.*2.0+p_2.*p_13.*p_16.*q_dot_5.*q_dot_6.*t21.*2.0+p_3.*p_14.*p_16.*q_dot_5.*q_dot_6.*t21.*2.0+p_4.*p_16.*p_19.*q_dot_2.*q_dot_5.*t21.*2.0+p_4.*p_16.*p_19.*q_dot_2.*q_dot_6.*t21.*2.0+p_4.*p_16.*p_19.*q_dot_5.*q_dot_6.*t21.*2.0;
et12 = t115+t118+t122+t124+t125+t129+t135+t139+t145+t176+t177+t178+u_1-p_2.*p_13.*p_18.*t6.*t12-p_2.*p_13.*p_18.*t6.*t13-p_3.*p_14.*p_17.*t6.*t12-p_3.*p_14.*p_17.*t6.*t13-p_2.*p_13.*p_18.*t6.*t17-p_4.*p_15.*p_19.*t6.*t12-p_3.*p_14.*p_17.*t6.*t17-p_4.*p_15.*p_19.*t6.*t13-p_4.*p_17.*p_19.*t6.*t12-p_4.*p_17.*p_19.*t6.*t13-p_4.*p_15.*p_19.*t6.*t17-p_4.*p_17.*p_19.*t6.*t17-p_2.*p_13.*p_18.*q_dot_2.*q_dot_3.*t6.*2.0-p_3.*p_14.*p_17.*q_dot_2.*q_dot_3.*t6.*2.0-p_2.*p_13.*p_18.*q_dot_2.*q_dot_7.*t6.*2.0-p_2.*p_13.*p_18.*q_dot_3.*q_dot_7.*t6.*2.0-p_3.*p_14.*p_17.*q_dot_2.*q_dot_7.*t6.*2.0-p_4.*p_15.*p_19.*q_dot_2.*q_dot_3.*t6.*2.0-p_3.*p_14.*p_17.*q_dot_3.*q_dot_7.*t6.*2.0-p_4.*p_17.*p_19.*q_dot_2.*q_dot_3.*t6.*2.0-p_4.*p_15.*p_19.*q_dot_2.*q_dot_7.*t6.*2.0-p_4.*p_15.*p_19.*q_dot_3.*q_dot_7.*t6.*2.0;
et13 = p_4.*p_17.*p_19.*q_dot_2.*q_dot_7.*t6.*-2.0-p_4.*p_17.*p_19.*q_dot_3.*q_dot_7.*t6.*2.0;
mt1 = [et1+et2+et3+et4;et5+et6+et7+et8+et9;et10+et11;t47+t48+t51+t52+t65+t66+t67+t68+t70+t72+t73+t74+t76+t78+t80+t83+t115+t118+t122+t124+t125+t129+t135+t139+t145+t153+t156+t157+t158+t159+t176+t177+t178+u_0+p_1.*p_12.*p_16.*t3.*t12+p_1.*p_12.*p_16.*t3.*t17+p_4.*p_15.*p_16.*t3.*t12+p_2.*p_16.*p_18.*t3.*t12+p_3.*p_16.*p_17.*t3.*t12+p_4.*p_16.*p_17.*t3.*t12+p_4.*p_15.*p_16.*t3.*t17+p_2.*p_16.*p_18.*t3.*t17+p_3.*p_16.*p_17.*t3.*t17+p_4.*p_16.*p_17.*t3.*t17+p_1.*p_12.*p_16.*q_dot_2.*q_dot_7.*t3.*2.0+p_4.*p_15.*p_16.*q_dot_2.*q_dot_7.*t3.*2.0+p_2.*p_16.*p_18.*q_dot_2.*q_dot_7.*t3.*2.0+p_3.*p_16.*p_17.*q_dot_2.*q_dot_7.*t3.*2.0+p_4.*p_16.*p_17.*q_dot_2.*q_dot_7.*t3.*2.0;et12+et13];
mt2 = [t49+t50+t53+t54+t69+t71+t75+t77+t79+t81+t82+t84+t100+t101+t102+t103+t104+t154+t155+t160+t183+t185+t188+u_2-p_1.*p_12.*p_16.*t4.*t12-p_4.*p_15.*p_16.*t4.*t12-p_2.*p_16.*p_18.*t4.*t12-p_3.*p_16.*p_17.*t4.*t12-p_4.*p_16.*p_17.*t4.*t12;t154+t155+t160+t183+t185+t188+u_3-p_2.*p_13.*p_18.*t7.*t12-p_3.*p_14.*p_17.*t7.*t12-p_2.*p_13.*p_18.*t7.*t15-p_3.*p_14.*p_17.*t7.*t15-p_4.*p_15.*p_19.*t7.*t12-p_4.*p_17.*p_19.*t7.*t12-p_4.*p_15.*p_19.*t7.*t15-p_4.*p_17.*p_19.*t7.*t15-p_2.*p_13.*p_18.*q_dot_2.*q_dot_5.*t7.*2.0-p_3.*p_14.*p_17.*q_dot_2.*q_dot_5.*t7.*2.0-p_4.*p_15.*p_19.*q_dot_2.*q_dot_5.*t7.*2.0-p_4.*p_17.*p_19.*q_dot_2.*q_dot_5.*t7.*2.0];
mt3 = [p22.*(-9.304e-1)+t47+t48+t51+t52+t65+t66+t67+t68+t70+t72+t73+t74+t76+t78+t80+t83+t96+t97+t98+t99+t105+t106+t107+t108+t109+t110+t111+t112+t113+t114+t148+t149+t150+t151+t152+t153+t156+t157+t158+t159+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t173+t174+t175+t176+t177+t178+t180+t181+t182+t184+t186+t187+t189-p22.*q_7-p23.*q_dot_7];
b = [mt1;mt2;mt3];
end

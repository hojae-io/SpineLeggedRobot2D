function A = A_fn(in1,in2)
%A_fn
%    A = A_fn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    01-Dec-2024 19:20:02

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
q_2 = in1(3,:);
q_3 = in1(4,:);
q_4 = in1(5,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_7 = in1(8,:);
t2 = cos(q_2);
t3 = cos(q_4);
t4 = cos(q_6);
t5 = sin(q_2);
t6 = sin(q_3);
t7 = sin(q_5);
t8 = p_2.*p_13;
t9 = p_3.*p_14;
t10 = p_10.*p_11;
t11 = p_4.*p_19;
t12 = q_2+q_5;
t13 = q_3+q_4;
t14 = q_2+q_7;
t15 = q_5+q_6;
t16 = p_1.*2.0;
t17 = p_2.*2.0;
t18 = p_3.*2.0;
t19 = p_4.*2.0;
t21 = p_12.^2;
t24 = p_15.^2;
t25 = p_16.^2;
t26 = p_17.^2;
t27 = p_18.^2;
t84 = p_5./8.0;
t29 = cos(t12);
t30 = cos(t14);
t31 = q_3+t14;
t32 = q_6+t12;
t33 = sin(t12);
t34 = sin(t13);
t35 = sin(t14);
t36 = sin(t15);
t37 = p_1.*p_16.*t2;
t38 = p_2.*p_16.*t2;
t39 = p_3.*p_16.*t2;
t40 = p_4.*p_16.*t2;
t43 = t13+t14;
t44 = p_1.*p_16.*t5;
t45 = p_2.*p_16.*t5;
t46 = p_3.*p_16.*t5;
t47 = p_4.*p_16.*t5;
t50 = p_1.*t21;
t51 = p_13.*t8;
t52 = p_1.*t25;
t53 = p_14.*t9;
t54 = p_2.*t25;
t55 = p_3.*t25;
t56 = p_4.*t24;
t57 = p_2.*t27;
t58 = p_3.*t26;
t59 = p_4.*t25;
t60 = p_4.*t26;
t61 = p_11.*t10;
t62 = p_19.*t11;
t63 = p_15.*p_17.*t19;
t64 = p_18.*t3.*t8;
t65 = p_17.*t3.*t9;
t66 = p_18.*t4.*t8;
t67 = p_17.*t4.*t9;
t68 = p_15.*t3.*t11;
t69 = p_15.*t4.*t11;
t70 = p_17.*t3.*t11;
t71 = p_17.*t4.*t11;
t73 = p_1.*p_12.*p_16.*t6;
t74 = p_1.*p_12.*p_16.*t7;
t75 = p_4.*p_15.*p_16.*t6;
t76 = p_2.*p_16.*p_18.*t6;
t77 = p_3.*p_16.*p_17.*t6;
t78 = p_4.*p_15.*p_16.*t7;
t79 = p_4.*p_16.*p_17.*t6;
t80 = p_2.*p_16.*p_18.*t7;
t81 = p_3.*p_16.*p_17.*t7;
t82 = p_4.*p_16.*p_17.*t7;
t105 = p_12.*p_16.*t6.*t16;
t106 = p_15.*p_16.*t6.*t19;
t107 = p_16.*p_18.*t6.*t17;
t108 = p_16.*p_17.*t6.*t18;
t109 = p_16.*p_17.*t6.*t19;
t170 = p_0.*p_16.*t2.*(3.0./1.0e+1);
t171 = p_0.*p_16.*t5.*(3.0./1.0e+1);
t193 = t8+t9+t11;
t197 = p_0.*t25.*(9.0./5.0e+1);
t201 = p_0+t16+t17+t18+t19;
t41 = cos(t31);
t42 = cos(t32);
t48 = sin(t31);
t49 = sin(t32);
t72 = cos(t43);
t83 = sin(t43);
t85 = -t37;
t86 = -t38;
t87 = -t39;
t88 = -t40;
t89 = p_15.*t29.*2.0;
t90 = p_17.*t29.*2.0;
t91 = p_16.*t30.*2.0;
t92 = p_18.*t29.*2.0;
t93 = t64.*2.0;
t94 = t65.*2.0;
t95 = t66.*2.0;
t96 = t67.*2.0;
t97 = t68.*2.0;
t98 = t69.*2.0;
t99 = t70.*2.0;
t100 = t71.*2.0;
t101 = p_15.*t33.*2.0;
t102 = p_17.*t33.*2.0;
t103 = p_16.*t35.*2.0;
t104 = p_18.*t33.*2.0;
t110 = p_1.*p_12.*t29;
t111 = p_1.*p_16.*t30;
t112 = p_4.*p_15.*t29;
t113 = p_2.*p_16.*t30;
t114 = p_2.*p_18.*t29;
t115 = p_3.*p_17.*t29;
t116 = p_3.*p_16.*t30;
t117 = p_4.*p_17.*t29;
t118 = p_4.*p_16.*t30;
t119 = p_1.*p_12.*t33;
t120 = p_1.*p_16.*t35;
t121 = p_4.*p_15.*t33;
t122 = p_2.*p_16.*t35;
t123 = p_2.*p_18.*t33;
t124 = p_3.*p_17.*t33;
t125 = p_3.*p_16.*t35;
t126 = p_4.*p_17.*t33;
t127 = p_4.*p_16.*t35;
t136 = p_16.*t8.*t34;
t137 = p_16.*t9.*t34;
t138 = p_16.*t8.*t36;
t139 = p_16.*t9.*t36;
t140 = p_16.*t11.*t34;
t141 = p_16.*t11.*t36;
t151 = -t74;
t152 = -t78;
t153 = -t80;
t154 = -t81;
t155 = -t82;
t192 = -t170;
t198 = p_0.*p_16.*t30.*(3.0./1.0e+1);
t199 = p_0.*p_16.*t35.*(3.0./1.0e+1);
t229 = p_7+p_8+t51+t53+t61+t62;
t248 = p_7+p_8+t10+t51+t53+t62+t64+t65+t68+t70;
t249 = p_7+p_8+t10+t51+t53+t62+t66+t67+t69+t71;
t128 = p_1.*p_12.*t41;
t129 = t8.*t42;
t130 = t9.*t42;
t131 = p_4.*p_15.*t41;
t132 = p_2.*p_18.*t41;
t133 = p_3.*p_17.*t41;
t134 = p_4.*p_17.*t41;
t135 = t11.*t42;
t142 = p_1.*p_12.*t48;
t143 = t8.*t49;
t144 = t9.*t49;
t145 = p_4.*p_15.*t48;
t146 = p_2.*p_18.*t48;
t147 = p_3.*p_17.*t48;
t148 = p_4.*p_17.*t48;
t149 = t11.*t49;
t150 = -t103;
t156 = p_12.*t41.*2.0;
t157 = p_13.*t42.*2.0;
t158 = p_14.*t42.*2.0;
t159 = p_15.*t41.*2.0;
t160 = p_17.*t41.*2.0;
t161 = p_18.*t41.*2.0;
t162 = p_19.*t42.*2.0;
t163 = p_12.*t48.*2.0;
t164 = p_13.*t49.*2.0;
t165 = p_14.*t49.*2.0;
t166 = p_15.*t48.*2.0;
t167 = p_17.*t48.*2.0;
t168 = p_18.*t48.*2.0;
t169 = p_19.*t49.*2.0;
t172 = p_13.*t72.*2.0;
t173 = p_14.*t72.*2.0;
t174 = p_19.*t72.*2.0;
t175 = t136.*2.0;
t176 = t137.*2.0;
t177 = t140.*2.0;
t178 = p_13.*t83.*2.0;
t179 = p_14.*t83.*2.0;
t180 = p_19.*t83.*2.0;
t181 = t8.*t72;
t182 = t9.*t72;
t183 = t11.*t72;
t184 = t8.*t83;
t185 = t9.*t83;
t186 = t11.*t83;
t188 = -t120;
t189 = -t122;
t190 = -t125;
t191 = -t127;
t194 = -t138;
t195 = -t139;
t196 = -t141;
t200 = -t199;
t202 = t42.*t193;
t203 = t49.*t193;
t209 = t72.*t193;
t210 = t83.*t193;
t250 = p_7+p_8+t51+t53+t62+t64+t65+t68+t70+t136+t137+t140;
t257 = p_6+p_7+p_8+p_9+t50+t51+t53+t56+t57+t58+t60+t62+t63+t73+t75+t76+t77+t79+t93+t94+t97+t99+t136+t137+t140;
t187 = -t156;
t204 = t90+t158;
t205 = t92+t157;
t206 = t91+t163;
t207 = t102+t165;
t208 = t104+t164;
t212 = t161+t172;
t213 = t160+t173;
t214 = t168+t178;
t215 = t167+t179;
t222 = t89+t90+t162;
t223 = t101+t102+t169;
t234 = t159+t160+t174;
t235 = t166+t167+t180;
t251 = p_7+p_8+t51+t53+t62+t66+t67+t69+t71+t194+t195+t196;
t259 = p_6+p_7+p_8+p_9+t50+t51+t53+t56+t57+t58+t60+t62+t63+t95+t96+t98+t100+t151+t152+t153+t154+t155+t194+t195+t196;
t260 = p_6+p_7+p_8+p_9+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t62+t63+t84+t93+t94+t97+t99+t105+t106+t107+t108+t109+t175+t176+t177+t197;
t261 = t44+t45+t46+t47+t110+t112+t114+t115+t117+t128+t129+t130+t131+t132+t133+t134+t135+t171+t181+t182+t183+t188+t189+t190+t191+t200;
t262 = t85+t86+t87+t88+t111+t113+t116+t118+t119+t121+t123+t124+t126+t142+t143+t144+t145+t146+t147+t148+t149+t184+t185+t186+t192+t198;
t211 = t103+t187;
t216 = (p_2.*t205)./2.0;
t217 = (p_3.*t204)./2.0;
t218 = (p_1.*t206)./2.0;
t219 = (p_2.*t208)./2.0;
t220 = (p_3.*t207)./2.0;
t224 = (p_2.*t212)./2.0;
t225 = (p_3.*t213)./2.0;
t226 = (p_2.*t214)./2.0;
t227 = (p_3.*t215)./2.0;
t230 = t91+t214;
t231 = t91+t215;
t232 = t150+t212;
t233 = t150+t213;
t236 = (p_4.*t222)./2.0;
t237 = (p_4.*t223)./2.0;
t240 = (p_4.*t234)./2.0;
t241 = (p_4.*t235)./2.0;
t244 = t91+t235;
t245 = t150+t234;
t221 = (p_1.*t211)./2.0;
t238 = (p_2.*t230)./2.0;
t239 = (p_3.*t231)./2.0;
t242 = (p_2.*t232)./2.0;
t243 = (p_3.*t233)./2.0;
t246 = (p_4.*t244)./2.0;
t247 = (p_4.*t245)./2.0;
t252 = t110+t216+t217+t236;
t253 = t119+t219+t220+t237;
t254 = t142+t226+t227+t241;
t255 = t128+t224+t225+t240;
t228 = -t221;
t256 = t198+t218+t238+t239+t246;
t258 = t200+t228+t242+t243+t247;
mt1 = [t201,0.0,t261,t255,t209,t252,t202,t258,0.0,t201,t262,t254,t210,t253,t203,t256,t261,t262,p_5./4.0+p_6.*2.0+p_7.*2.0+p_8.*2.0+p_9.*2.0+t51.*2.0+t53.*2.0+t62.*2.0-t74.*2.0-t78.*2.0-t80.*2.0-t81.*2.0-t82.*2.0+t93+t94+t95+t96+t97+t98+t99+t100+t105+t106+t107+t108+t109-t138.*2.0-t139.*2.0-t141.*2.0+t175+t176+t177+p_0.*t25.*(9.0./2.5e+1)+t16.*t21+t16.*t25+t17.*t25+t18.*t25+t19.*t24+t17.*t27+t18.*t26+t19.*t25+t19.*t26+p_4.*p_15.*p_17.*4.0,t257,t250,t259,t251,t260,t255,t254,t257,p_6+p_9+p_10+t50+t56+t57+t58+t60+t63+t93+t94+t97+t99+t229,t248,0.0,0.0,t257,t209,t210,t250,t248,t229,0.0,0.0,t250,t252,t253,t259,0.0,0.0,p_6+p_9+p_10+t50+t56+t57+t58+t60+t63+t95+t96+t98+t100+t229,t249,0.0,t202,t203,t251,0.0,0.0,t249,t229,0.0,t258,t256];
mt2 = [t260,t257,t250,0.0,0.0,t260];
A = reshape([mt1,mt2],8,8);
end

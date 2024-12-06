function InvDynamics = InvDynamics_fn(in1,in2,in3,in4)
%InvDynamics_fn
%    InvDynamics = InvDynamics_fn(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    25-Nov-2024 00:07:52

p_0 = in4(1,:);
p_1 = in4(2,:);
p_2 = in4(3,:);
p_3 = in4(4,:);
p_4 = in4(5,:);
p_5 = in4(6,:);
p_6 = in4(7,:);
p_7 = in4(8,:);
p_8 = in4(9,:);
p_9 = in4(10,:);
p_10 = in4(11,:);
p_11 = in4(12,:);
p_12 = in4(13,:);
p_13 = in4(14,:);
p_14 = in4(15,:);
p_15 = in4(16,:);
p_16 = in4(17,:);
p_17 = in4(18,:);
p_18 = in4(19,:);
p_19 = in4(20,:);
p_21 = in4(22,:);
q_2 = in1(3,:);
q_3 = in1(4,:);
q_4 = in1(5,:);
q_5 = in1(6,:);
q_6 = in1(7,:);
q_dot_0 = in2(1,:);
q_dot_1 = in2(2,:);
q_dot_2 = in2(3,:);
q_dot_3 = in2(4,:);
q_dot_4 = in2(5,:);
q_dot_5 = in2(6,:);
q_dot_6 = in2(7,:);
q_ddot_0 = in3(1,:);
q_ddot_1 = in3(2,:);
q_ddot_2 = in3(3,:);
q_ddot_3 = in3(4,:);
q_ddot_4 = in3(5,:);
q_ddot_5 = in3(6,:);
q_ddot_6 = in3(7,:);
t2 = cos(q_2);
t3 = sin(q_2);
t4 = p_10.*p_11;
t5 = q_2+q_3;
t6 = q_2+q_5;
t7 = p_1.*2.0;
t8 = p_2.*2.0;
t9 = p_3.*2.0;
t10 = p_4.*2.0;
t12 = p_12.^2;
t13 = p_13.^2;
t14 = p_14.^2;
t15 = p_19.^2;
t16 = p_16.*t2;
t17 = cos(t5);
t18 = cos(t6);
t19 = q_4+t5;
t20 = q_6+t6;
t21 = p_16.*t3;
t22 = sin(t5);
t23 = sin(t6);
t28 = p_11.*t4;
t187 = p_0+t7+t8+t9+t10;
t24 = cos(t19);
t25 = cos(t20);
t26 = sin(t19);
t27 = sin(t20);
t29 = t16.*2.0;
t30 = t21.*2.0;
t31 = p_12.*t17;
t32 = p_12.*t18;
t33 = p_15.*t17;
t34 = p_15.*t18;
t35 = p_17.*t17;
t36 = p_18.*t17;
t37 = p_17.*t18;
t38 = p_18.*t18;
t39 = p_12.*t22;
t40 = p_12.*t23;
t41 = p_15.*t22;
t42 = p_15.*t23;
t43 = p_17.*t22;
t44 = p_18.*t22;
t45 = p_17.*t23;
t46 = p_18.*t23;
t47 = -t16;
t49 = -t21;
t48 = -t29;
t50 = -t30;
t51 = t31.*2.0;
t52 = t32.*2.0;
t53 = t33.*2.0;
t54 = t34.*2.0;
t55 = t35.*2.0;
t56 = t36.*2.0;
t57 = t37.*2.0;
t58 = t38.*2.0;
t59 = t24.^2;
t60 = t25.^2;
t61 = t39.*2.0;
t62 = t40.*2.0;
t63 = t41.*2.0;
t64 = t42.*2.0;
t65 = t43.*2.0;
t66 = t44.*2.0;
t67 = t45.*2.0;
t68 = t46.*2.0;
t69 = t26.^2;
t70 = t27.^2;
t71 = p_1.*t31;
t72 = p_1.*t32;
t73 = p_13.*t24;
t74 = p_14.*t24;
t75 = p_13.*t25;
t76 = p_14.*t25;
t77 = p_19.*t24;
t78 = p_19.*t25;
t79 = q_dot_2.*t31;
t80 = q_dot_3.*t31;
t81 = q_dot_2.*t32;
t82 = q_dot_5.*t32;
t83 = p_1.*t39;
t84 = p_1.*t40;
t85 = p_4.*t41;
t86 = p_2.*t44;
t87 = p_3.*t43;
t88 = p_4.*t42;
t89 = p_4.*t43;
t90 = p_2.*t46;
t91 = p_3.*t45;
t92 = p_4.*t45;
t93 = p_13.*t26;
t94 = p_14.*t26;
t95 = p_13.*t27;
t96 = p_14.*t27;
t97 = p_19.*t26;
t98 = p_19.*t27;
t99 = q_dot_2.*t39;
t100 = q_dot_3.*t39;
t101 = q_dot_2.*t40;
t102 = q_dot_5.*t40;
t179 = t16+t39;
t180 = t21+t32;
t202 = -q_dot_2.*(t21-t31);
t203 = -q_dot_2.*(t16-t40);
t246 = t40.*(t16-t40).*-2.0;
t259 = t31.*(t21-t31).*-2.0;
t103 = p_2.*t73;
t104 = p_3.*t74;
t105 = p_2.*t75;
t106 = p_3.*t76;
t107 = p_4.*t77;
t108 = p_4.*t78;
t109 = q_dot_2.*t73;
t110 = q_dot_3.*t73;
t111 = q_dot_2.*t74;
t112 = q_dot_4.*t73;
t113 = q_dot_3.*t74;
t114 = q_dot_4.*t74;
t115 = q_dot_2.*t75;
t116 = q_dot_2.*t76;
t117 = q_dot_2.*t77;
t118 = q_dot_5.*t75;
t119 = q_dot_3.*t77;
t120 = q_dot_6.*t75;
t121 = q_dot_5.*t76;
t122 = q_dot_4.*t77;
t123 = q_dot_6.*t76;
t124 = q_dot_2.*t78;
t125 = q_dot_5.*t78;
t126 = q_dot_6.*t78;
t127 = p_2.*t93;
t128 = p_3.*t94;
t129 = p_2.*t95;
t130 = p_3.*t96;
t131 = p_4.*t97;
t132 = p_4.*t98;
t133 = q_dot_2.*t93;
t134 = q_dot_3.*t93;
t135 = q_dot_2.*t94;
t136 = q_dot_4.*t93;
t137 = q_dot_3.*t94;
t138 = q_dot_4.*t94;
t139 = q_dot_2.*t95;
t140 = q_dot_2.*t96;
t141 = q_dot_2.*t97;
t142 = q_dot_5.*t95;
t143 = q_dot_3.*t97;
t144 = q_dot_6.*t95;
t145 = q_dot_5.*t96;
t146 = q_dot_4.*t97;
t147 = q_dot_6.*t96;
t148 = q_dot_2.*t98;
t149 = q_dot_5.*t98;
t150 = q_dot_6.*t98;
t151 = t73.*2.0;
t152 = t74.*2.0;
t153 = t75.*2.0;
t154 = t76.*2.0;
t155 = t77.*2.0;
t156 = t78.*2.0;
t157 = q_dot_3.*t51;
t158 = q_dot_5.*t52;
t159 = t93.*2.0;
t160 = t94.*2.0;
t161 = t95.*2.0;
t162 = t96.*2.0;
t163 = t97.*2.0;
t164 = t98.*2.0;
t165 = q_dot_3.*t61;
t166 = q_dot_5.*t62;
t181 = q_dot_2.*t179;
t182 = q_dot_2.*t180;
t185 = t29+t61;
t186 = t30+t52;
t190 = t35+t74;
t191 = t36+t73;
t192 = t37+t76;
t193 = t38+t75;
t194 = t43+t94;
t195 = t44+t93;
t196 = t45+t96;
t197 = t46+t95;
t198 = t79+t80;
t199 = t81+t82;
t200 = t99+t100;
t201 = t101+t102;
t236 = t61.*t179;
t243 = t52.*t180;
t244 = p_1.*(t30-t51).*(-1.0./2.0);
t245 = p_1.*(t29-t62).*(-1.0./2.0);
t247 = t33+t35+t77;
t248 = t34+t37+t78;
t253 = t41+t43+t97;
t254 = t42+t45+t98;
t308 = t80+t202;
t309 = t102+t203;
t167 = t112.*2.0;
t168 = t114.*2.0;
t169 = t120.*2.0;
t170 = t122.*2.0;
t171 = t123.*2.0;
t172 = t126.*2.0;
t173 = t136.*2.0;
t174 = t138.*2.0;
t175 = t144.*2.0;
t176 = t146.*2.0;
t177 = t147.*2.0;
t178 = t150.*2.0;
t204 = q_dot_2.*t190;
t205 = q_dot_2.*t191;
t206 = q_dot_3.*t190;
t207 = q_dot_3.*t191;
t208 = q_dot_2.*t192;
t209 = q_dot_2.*t193;
t210 = q_dot_5.*t192;
t211 = q_dot_5.*t193;
t212 = q_dot_2.*t194;
t213 = q_dot_2.*t195;
t214 = q_dot_3.*t194;
t215 = q_dot_3.*t195;
t216 = q_dot_2.*t196;
t217 = q_dot_2.*t197;
t218 = q_dot_5.*t196;
t219 = q_dot_5.*t197;
t220 = t55+t152;
t221 = t56+t151;
t222 = t57+t154;
t223 = t58+t153;
t224 = t65+t160;
t225 = t66+t159;
t226 = t67+t162;
t227 = t68+t161;
t237 = (p_1.*t185)./2.0;
t238 = (p_1.*t186)./2.0;
t239 = t21+t192;
t240 = t21+t193;
t241 = t16+t194;
t242 = t16+t195;
t255 = t49+t190;
t256 = t49+t191;
t257 = t47+t196;
t258 = t47+t197;
t268 = t82+t182;
t273 = t100+t181;
t274 = q_dot_2.*t247;
t275 = q_dot_3.*t247;
t276 = q_dot_2.*t248;
t277 = q_dot_5.*t248;
t280 = q_dot_2.*t253;
t281 = q_dot_3.*t253;
t282 = q_dot_2.*t254;
t283 = q_dot_5.*t254;
t292 = t151.*t191;
t293 = t152.*t190;
t294 = t153.*t193;
t295 = t154.*t192;
t296 = t159.*t195;
t297 = t160.*t194;
t298 = t161.*t197;
t299 = t162.*t196;
t302 = t53+t55+t155;
t303 = t54+t57+t156;
t306 = t63+t65+t163;
t307 = t64+t67+t164;
t310 = t16+t253;
t311 = q_dot_0+t308;
t312 = q_dot_1+t309;
t313 = t21+t248;
t314 = t103+t104+t107;
t315 = t105+t106+t108;
t316 = t109+t110+t112;
t317 = t111+t113+t114;
t318 = t115+t118+t120;
t319 = t117+t119+t122;
t320 = t116+t121+t123;
t321 = t124+t125+t126;
t322 = t127+t128+t131;
t323 = t129+t130+t132;
t324 = t133+t134+t136;
t325 = t135+t137+t138;
t326 = t139+t142+t144;
t327 = t141+t143+t146;
t328 = t140+t145+t147;
t329 = t148+t149+t150;
t330 = t47+t254;
t341 = t49+t247;
t349 = t155.*t247;
t350 = t156.*t248;
t355 = t163.*t253;
t356 = t164.*t254;
t386 = t236+t259;
t387 = t243+t246;
t228 = t206.*2.0;
t229 = t207.*2.0;
t230 = t210.*2.0;
t231 = t211.*2.0;
t232 = t214.*2.0;
t233 = t215.*2.0;
t234 = t218.*2.0;
t235 = t219.*2.0;
t249 = q_dot_2.*t239;
t250 = q_dot_2.*t240;
t251 = q_dot_2.*t241;
t252 = q_dot_2.*t242;
t260 = t30+t222;
t261 = t30+t223;
t262 = t29+t224;
t263 = t29+t225;
t264 = (p_2.*t221)./2.0;
t265 = (p_3.*t220)./2.0;
t266 = (p_2.*t223)./2.0;
t267 = (p_3.*t222)./2.0;
t269 = (p_2.*t225)./2.0;
t270 = (p_3.*t224)./2.0;
t271 = (p_2.*t227)./2.0;
t272 = (p_3.*t226)./2.0;
t278 = q_dot_0+t268;
t279 = q_dot_1+t273;
t284 = q_dot_2.*t255;
t285 = q_dot_2.*t256;
t286 = q_dot_2.*t257;
t287 = q_dot_2.*t258;
t288 = t50+t220;
t289 = t50+t221;
t290 = t48+t226;
t291 = t48+t227;
t300 = t275.*2.0;
t301 = t277.*2.0;
t304 = t281.*2.0;
t305 = t283.*2.0;
t335 = t153.*t240;
t336 = t154.*t239;
t337 = t159.*t242;
t338 = t160.*t241;
t339 = q_dot_2.*t313;
t340 = q_dot_2.*t310;
t346 = (p_4.*t302)./2.0;
t347 = (p_4.*t303)./2.0;
t348 = t29+t306;
t351 = (p_4.*t306)./2.0;
t352 = (p_4.*t307)./2.0;
t353 = t151.*t256;
t354 = t152.*t255;
t357 = t161.*t258;
t358 = t162.*t257;
t359 = q_dot_2.*t341;
t360 = q_dot_2.*t330;
t361 = t30+t303;
t362 = t48+t307;
t365 = t50+t302;
t369 = t39.*t311.*-2.0;
t370 = t156.*t313;
t371 = t163.*t310;
t374 = t192.*t239.*2.0;
t375 = t193.*t240.*2.0;
t377 = t194.*t241.*2.0;
t378 = t195.*t242.*2.0;
t379 = t155.*t341;
t381 = t164.*t330;
t382 = t190.*t255.*2.0;
t383 = t191.*t256.*2.0;
t384 = t196.*t257.*2.0;
t385 = t197.*t258.*2.0;
t388 = t136+t213+t215;
t389 = t138+t212+t214;
t390 = t144+t217+t219;
t391 = t147+t216+t218;
t392 = t112+t205+t207;
t393 = t114+t204+t206;
t394 = t120+t209+t211;
t395 = t123+t208+t210;
t396 = (p_1.*t386)./2.0;
t397 = (p_1.*t387)./2.0;
t402 = t292+t296;
t403 = t293+t297;
t404 = t294+t298;
t405 = t295+t299;
t408 = t248.*t313.*2.0;
t409 = t253.*t310.*2.0;
t418 = t247.*t341.*2.0;
t419 = t254.*t330.*2.0;
t426 = t146+t280+t281;
t427 = t150+t282+t283;
t428 = t122+t274+t275;
t429 = t126+t276+t277;
t445 = t349+t355;
t446 = t350+t356;
t331 = (p_2.*t261)./2.0;
t332 = (p_3.*t260)./2.0;
t333 = (p_2.*t263)./2.0;
t334 = (p_3.*t262)./2.0;
t342 = (p_2.*t289)./2.0;
t343 = (p_3.*t288)./2.0;
t344 = (p_2.*t291)./2.0;
t345 = (p_3.*t290)./2.0;
t363 = t51.*t279;
t364 = t62.*t278;
t372 = (p_4.*t361)./2.0;
t373 = (p_4.*t348)./2.0;
t376 = (p_4.*t365)./2.0;
t380 = (p_4.*t362)./2.0;
t398 = t120+t211+t250;
t399 = t123+t210+t249;
t400 = t136+t215+t252;
t401 = t138+t214+t251;
t412 = t112+t207+t285;
t413 = t114+t206+t284;
t414 = t144+t219+t287;
t415 = t147+t218+t286;
t422 = (p_2.*t402)./2.0;
t423 = (p_3.*t403)./2.0;
t424 = (p_2.*t404)./2.0;
t425 = (p_3.*t405)./2.0;
t434 = t337+t353;
t435 = t338+t354;
t436 = t335+t357;
t437 = t336+t358;
t444 = t126+t277+t339;
t447 = t146+t281+t340;
t452 = t122+t275+t359;
t453 = t150+t283+t360;
t464 = (p_4.*t445)./2.0;
t465 = (p_4.*t446)./2.0;
t477 = t371+t379;
t478 = t370+t381;
t484 = t377+t382;
t485 = t378+t383;
t486 = t374+t384;
t487 = t375+t385;
t494 = t71+t264+t265+t346;
t495 = t72+t266+t267+t347;
t496 = t83+t269+t270+t351;
t497 = t84+t271+t272+t352;
t504 = t409+t418;
t505 = t408+t419;
t406 = q_dot_1+t400;
t407 = q_dot_1+t401;
t410 = q_dot_0+t398;
t411 = q_dot_0+t399;
t416 = q_dot_1+t414;
t417 = q_dot_1+t415;
t420 = q_dot_0+t412;
t421 = q_dot_0+t413;
t450 = q_dot_1+t447;
t451 = q_dot_0+t444;
t454 = q_dot_1+t453;
t455 = q_dot_0+t452;
t456 = (p_2.*t434)./2.0;
t457 = (p_3.*t435)./2.0;
t458 = (p_2.*t436)./2.0;
t459 = (p_3.*t437)./2.0;
t488 = (p_4.*t477)./2.0;
t489 = (p_4.*t478)./2.0;
t490 = (p_2.*t485)./2.0;
t491 = (p_3.*t484)./2.0;
t492 = (p_2.*t487)./2.0;
t493 = (p_3.*t486)./2.0;
t506 = (p_4.*t504)./2.0;
t507 = (p_4.*t505)./2.0;
InvDynamics = ft_1({p_1,p_10,p_2,p_21,p_3,p_4,p_5,p_6,p_7,p_8,p_9,q_ddot_0,q_ddot_1,q_ddot_2,q_ddot_3,q_ddot_4,q_ddot_5,q_ddot_6,q_dot_2,q_dot_3,q_dot_4,q_dot_5,q_dot_6,t109,t110,t111,t113,t115,t116,t117,t118,t119,t12,t121,t124,t125,t13,t133,t134,t135,t137,t139,t14,t140,t141,t142,t143,t145,t148,t149,t15,t151,t152,t153,t154,t155,t156,t157,t158,t159,t16,t160,t161,t162,t163,t164,t165,t166,t167,t168,t169,t17,t170,t171,t172,t173,t174,t175,t176,t177,t178,t179,t18,t180,t181,t182,t187,t190,t191,t192,t193,t194,t195,t196,t197,t198,t199,t200,t201,t204,t205,t208,t209,t21,t212,t213,t216,t217,t22,t228,t229,t23,t230,t231,t232,t233,t234,t235,t237,t238,t239,t240,t241,t242,t244,t245,t247,t248,t249,t250,t251,t252,t253,t254,t255,t256,t257,t258,t268,t273,t274,t276,t278,t279,t28,t280,t282,t284,t285,t286,t287,t300,t301,t304,t305,t308,t309,t31,t310,t311,t312,t313,t314,t315,t316,t317,t318,t319,t32,t320,t321,t322,t323,t324,t325,t326,t327,t328,t329,t330,t331,t332,t333,t334,t339,t340,t341,t342,t343,t344,t345,t359,t360,t363,t364,t369,t372,t373,t376,t380,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t40,t400,t401,t406,t407,t410,t411,t412,t413,t414,t415,t416,t417,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t444,t447,t450,t451,t452,t453,t454,t455,t456,t457,t458,t459,t464,t465,t488,t489,t490,t491,t492,t493,t494,t495,t496,t497,t506,t507,t51,t52,t59,t60,t61,t62,t69,t70,t73,t74,t75,t76,t77,t78,t83,t84,t85,t86,t87,t88,t89,t90,t91,t92,t93,t94,t95,t96,t97,t98});
end
function InvDynamics = ft_1(ct)
[p_1,p_10,p_2,p_21,p_3,p_4,p_5,p_6,p_7,p_8,p_9,q_ddot_0,q_ddot_1,q_ddot_2,q_ddot_3,q_ddot_4,q_ddot_5,q_ddot_6,q_dot_2,q_dot_3,q_dot_4,q_dot_5,q_dot_6,t109,t110,t111,t113,t115,t116,t117,t118,t119,t12,t121,t124,t125,t13,t133,t134,t135,t137,t139,t14,t140,t141,t142,t143,t145,t148,t149,t15,t151,t152,t153,t154,t155,t156,t157,t158,t159,t16,t160,t161,t162,t163,t164,t165,t166,t167,t168,t169,t17,t170,t171,t172,t173,t174,t175,t176,t177,t178,t179,t18,t180,t181,t182,t187,t190,t191,t192,t193,t194,t195,t196,t197,t198,t199,t200,t201,t204,t205,t208,t209,t21,t212,t213,t216,t217,t22,t228,t229,t23,t230,t231,t232,t233,t234,t235,t237,t238,t239,t240,t241,t242,t244,t245,t247,t248,t249,t250,t251,t252,t253,t254,t255,t256,t257,t258,t268,t273,t274,t276,t278,t279,t28,t280,t282,t284,t285,t286,t287,t300,t301,t304,t305,t308,t309,t31,t310,t311,t312,t313,t314,t315,t316,t317,t318,t319,t32,t320,t321,t322,t323,t324,t325,t326,t327,t328,t329,t330,t331,t332,t333,t334,t339,t340,t341,t342,t343,t344,t345,t359,t360,t363,t364,t369,t372,t373,t376,t380,t388,t389,t390,t391,t392,t393,t394,t395,t396,t397,t398,t399,t4,t40,t400,t401,t406,t407,t410,t411,t412,t413,t414,t415,t416,t417,t420,t421,t422,t423,t424,t425,t426,t427,t428,t429,t444,t447,t450,t451,t452,t453,t454,t455,t456,t457,t458,t459,t464,t465,t488,t489,t490,t491,t492,t493,t494,t495,t496,t497,t506,t507,t51,t52,t59,t60,t61,t62,t69,t70,t73,t74,t75,t76,t77,t78,t83,t84,t85,t86,t87,t88,t89,t90,t91,t92,t93,t94,t95,t96,t97,t98] = ct{:};
t508 = p_7+p_8+t4+t422+t423+t464;
t509 = p_7+p_8+t4+t424+t425+t465;
t512 = t238+t244+t331+t332+t342+t343+t372+t376;
t513 = t237+t245+t333+t334+t344+t345+t373+t380;
t430 = t151.*t406;
t431 = t152.*t407;
t432 = t161.*t410;
t433 = t162.*t411;
t440 = t153.*t416;
t441 = t154.*t417;
t442 = t159.*t420;
t443 = t160.*t421;
t448 = t93.*t420.*-2.0;
t449 = t94.*t421.*-2.0;
t460 = t197.*t410.*2.0;
t461 = t196.*t411.*2.0;
t462 = t191.*t406.*2.0;
t463 = t190.*t407.*2.0;
t468 = t195.*t420.*2.0;
t469 = t194.*t421.*2.0;
t470 = t193.*t416.*2.0;
t471 = t192.*t417.*2.0;
t474 = t155.*t450;
t475 = t164.*t451;
t480 = t156.*t454;
t481 = t163.*t455;
t482 = t78.*t454.*-2.0;
t483 = t97.*t455.*-2.0;
t498 = t254.*t451.*2.0;
t499 = t247.*t450.*2.0;
t501 = t253.*t455.*2.0;
t502 = t248.*t454.*2.0;
t510 = p_7+p_8+t456+t457+t488;
t511 = p_7+p_8+t458+t459+t489;
t514 = p_6+p_7+p_8+p_9+t396+t490+t491+t506;
t515 = p_6+p_7+p_8+p_9+t397+t492+t493+t507;
t472 = -t470;
t473 = -t471;
t503 = -t501;
et1 = -q_dot_4.*((p_2.*(t133.*2.0+t134.*2.0+t173))./2.0+(p_3.*(t135.*2.0+t137.*2.0+t174))./2.0+(p_4.*(t141.*2.0+t143.*2.0+t176))./2.0)-q_dot_6.*((p_2.*(t139.*2.0+t142.*2.0+t175))./2.0+(p_3.*(t140.*2.0+t145.*2.0+t177))./2.0+(p_4.*(t148.*2.0+t149.*2.0+t178))./2.0)-q_dot_2.*((p_1.*(t166-q_dot_2.*(t16-t40).*2.0))./2.0+(p_1.*(t165+t181.*2.0))./2.0+(p_2.*(t173+t233+t252.*2.0))./2.0+(p_3.*(t174+t232+t251.*2.0))./2.0+(p_2.*(t175+t235+t287.*2.0))./2.0+(p_3.*(t177+t234+t286.*2.0))./2.0+(p_4.*(t176+t304+t340.*2.0))./2.0+(p_4.*(t178+t305+t360.*2.0))./2.0)+q_ddot_0.*t187+q_ddot_4.*t314+q_ddot_6.*t315+q_ddot_3.*t494;
et2 = q_ddot_5.*t495+q_ddot_2.*t512-q_dot_3.*((p_1.*(t165+q_dot_2.*t61))./2.0+(p_2.*(t173+t213.*2.0+t233))./2.0+(p_3.*(t174+t212.*2.0+t232))./2.0+(p_4.*(t176+t280.*2.0+t304))./2.0)-q_dot_5.*((p_1.*(t166+q_dot_2.*t62))./2.0+(p_2.*(t175+t217.*2.0+t235))./2.0+(p_3.*(t177+t216.*2.0+t234))./2.0+(p_4.*(t178+t282.*2.0+t305))./2.0);
et3 = q_dot_4.*((p_2.*(t109.*2.0+t110.*2.0+t167))./2.0+(p_3.*(t111.*2.0+t113.*2.0+t168))./2.0+(p_4.*(t117.*2.0+t119.*2.0+t170))./2.0)+q_dot_6.*((p_2.*(t115.*2.0+t118.*2.0+t169))./2.0+(p_3.*(t116.*2.0+t121.*2.0+t171))./2.0+(p_4.*(t124.*2.0+t125.*2.0+t172))./2.0)+q_dot_2.*((p_1.*(t157-q_dot_2.*(t21-t31).*2.0))./2.0+(p_1.*(t158+t182.*2.0))./2.0+(p_2.*(t169+t231+t250.*2.0))./2.0+(p_3.*(t171+t230+t249.*2.0))./2.0+(p_2.*(t167+t229+t285.*2.0))./2.0+(p_3.*(t168+t228+t284.*2.0))./2.0+(p_4.*(t172+t301+t339.*2.0))./2.0+(p_4.*(t170+t300+t359.*2.0))./2.0)+p_21.*t187+q_ddot_1.*t187+q_ddot_4.*t322+q_ddot_6.*t323+q_ddot_3.*t496+q_ddot_5.*t497;
et4 = q_ddot_2.*t513+q_dot_3.*((p_1.*(t157+q_dot_2.*t51))./2.0+(p_2.*(t167+t205.*2.0+t229))./2.0+(p_3.*(t168+t204.*2.0+t228))./2.0+(p_4.*(t170+t274.*2.0+t300))./2.0)+q_dot_5.*((p_1.*(t158+q_dot_2.*t52))./2.0+(p_2.*(t169+t209.*2.0+t231))./2.0+(p_3.*(t171+t208.*2.0+t230))./2.0+(p_4.*(t172+t276.*2.0+t301))./2.0);
et5 = (p_2.*(t240.*t414.*2.0-t240.*t416.*2.0-t258.*t398.*2.0+t258.*t410.*2.0))./2.0-(p_2.*(t242.*t412.*2.0-t256.*t400.*2.0-t242.*t420.*2.0+t256.*t406.*2.0))./2.0+(p_3.*(t239.*t415.*2.0-t239.*t417.*2.0-t257.*t399.*2.0+t257.*t411.*2.0))./2.0-(p_3.*(t241.*t413.*2.0-t255.*t401.*2.0-t241.*t421.*2.0+t255.*t407.*2.0))./2.0+(p_4.*(t313.*t453.*2.0-t313.*t454.*2.0-t330.*t444.*2.0+t330.*t451.*2.0))./2.0-(p_4.*(t310.*t452.*2.0-t310.*t455.*2.0-t341.*t447.*2.0+t341.*t450.*2.0))./2.0-(p_1.*(t179.*t308.*2.0-t179.*t311.*2.0+t273.*(t21-t31).*2.0-t279.*(t21-t31).*2.0))./2.0;
et6 = (p_1.*(t180.*t309.*2.0-t180.*t312.*2.0+t268.*(t16-t40).*2.0-t278.*(t16-t40).*2.0))./2.0;
et7 = -q_dot_5.*((p_4.*(t498-t502+t313.*t427.*2.0-t330.*t429.*2.0))./2.0+(p_2.*(t460+t472+t240.*t390.*2.0-t258.*t394.*2.0))./2.0+(p_3.*(t461+t473+t239.*t391.*2.0-t257.*t395.*2.0))./2.0+(p_1.*(t364-t32.*t312.*2.0+t180.*t201.*2.0+t199.*(t16-t40).*2.0))./2.0)+q_ddot_0.*t512+q_ddot_1.*t513+q_ddot_4.*t510+q_ddot_3.*t514+q_ddot_6.*t511+q_ddot_5.*t515-q_dot_2.*(et5+et6)-q_dot_6.*((p_2.*(t432-t75.*t416.*2.0+t240.*t326.*2.0-t258.*t318.*2.0))./2.0+(p_3.*(t433-t76.*t417.*2.0+t239.*t328.*2.0-t257.*t320.*2.0))./2.0-(p_4.*(t480-t98.*t451.*2.0-t313.*t329.*2.0+t321.*t330.*2.0))./2.0)+p_21.*(t83+t84+t85+t86+t87+t88+t89+t90+t91+t92+t322+t323);
et8 = q_dot_3.*((p_2.*(t462-t468+t242.*t392.*2.0-t256.*t388.*2.0))./2.0+(p_3.*(t463-t469+t241.*t393.*2.0-t255.*t389.*2.0))./2.0+(p_4.*(t499+t503+t310.*t428.*2.0-t341.*t426.*2.0))./2.0+(p_1.*(t363+t369+t179.*t198.*2.0+t200.*(t21-t31).*2.0))./2.0)-(p_1.*(t268.*t312.*2.0-t278.*t309.*2.0))./2.0+(p_1.*(t273.*t311.*2.0-t279.*t308.*2.0))./2.0-(p_2.*(t398.*t416.*2.0-t410.*t414.*2.0))./2.0-(p_2.*(t406.*t412.*2.0-t400.*t420.*2.0))./2.0-(p_3.*(t399.*t417.*2.0-t411.*t415.*2.0))./2.0-(p_3.*(t407.*t413.*2.0-t401.*t421.*2.0))./2.0-(p_4.*(t444.*t454.*2.0-t451.*t453.*2.0))./2.0;
et9 = (p_4.*(t447.*t455.*2.0-t450.*t452.*2.0))./2.0-q_dot_4.*((p_2.*(t442-t73.*t406.*2.0-t242.*t316.*2.0+t256.*t324.*2.0))./2.0+(p_3.*(t443-t74.*t407.*2.0-t241.*t317.*2.0+t255.*t325.*2.0))./2.0-(p_4.*(t474+t483+t310.*t319.*2.0-t327.*t341.*2.0))./2.0);
et10 = q_ddot_2.*(p_5+p_6.*2.0+p_7.*2.0+p_8.*2.0+p_9.*2.0+(p_3.*(t239.^2.*2.0+t257.^2.*2.0))./2.0+(p_3.*(t241.^2.*2.0+t255.^2.*2.0))./2.0+(p_2.*(t240.^2.*2.0+t258.^2.*2.0))./2.0+(p_2.*(t242.^2.*2.0+t256.^2.*2.0))./2.0+(p_4.*(t313.^2.*2.0+t330.^2.*2.0))./2.0+(p_4.*(t310.^2.*2.0+t341.^2.*2.0))./2.0+(p_1.*((t21-t31).^2.*2.0+t179.^2.*2.0))./2.0+(p_1.*((t16-t40).^2.*2.0+t180.^2.*2.0))./2.0);
et11 = q_ddot_3.*(p_6+p_7+p_8+p_9+p_10+t28+(p_1.*(t12.*t17.^2.*2.0+t12.*t22.^2.*2.0))./2.0+(p_3.*(t190.^2.*2.0+t194.^2.*2.0))./2.0+(p_2.*(t191.^2.*2.0+t195.^2.*2.0))./2.0+(p_4.*(t247.^2.*2.0+t253.^2.*2.0))./2.0)+q_ddot_0.*t494+q_ddot_1.*t496+q_ddot_4.*t508+q_ddot_2.*t514+q_dot_3.*((p_2.*(t462-t468-t191.*t388.*2.0+t195.*t392.*2.0))./2.0+(p_3.*(t463-t469-t190.*t389.*2.0+t194.*t393.*2.0))./2.0+(p_1.*(t363+t369-t31.*t200.*2.0+t61.*t198))./2.0+(p_4.*(t499+t503-t247.*t426.*2.0+t253.*t428.*2.0))./2.0);
et12 = q_dot_2.*((p_2.*(t462-t468-t191.*t400.*2.0+t195.*t412.*2.0))./2.0+(p_3.*(t463-t469-t190.*t401.*2.0+t194.*t413.*2.0))./2.0+(p_1.*(t363+t369-t31.*t273.*2.0+t61.*t308))./2.0+(p_4.*(t499+t503-t247.*t447.*2.0+t253.*t452.*2.0))./2.0)-(p_1.*(t198.*t279.*2.0-t200.*t311.*2.0))./2.0-(p_2.*(t392.*t406.*2.0-t388.*t420.*2.0))./2.0-(p_3.*(t393.*t407.*2.0-t389.*t421.*2.0))./2.0-(p_4.*(t428.*t450.*2.0-t426.*t455.*2.0))./2.0+p_21.*(t83+t85+t86+t87+t89+t322);
et13 = q_dot_4.*(p_4.*(t481-t77.*t450.*2.0-t253.*t319.*2.0+t247.*t327.*2.0).*(-1.0./2.0)+(p_2.*(t430+t448+t195.*t316.*2.0-t191.*t324.*2.0))./2.0+(p_3.*(t431+t449+t194.*t317.*2.0-t190.*t325.*2.0))./2.0);
et14 = q_dot_4.*((p_2.*(t430+t448-t73.*t324.*2.0+t159.*t316))./2.0+(p_3.*(t431+t449-t74.*t325.*2.0+t160.*t317))./2.0+(p_4.*(t474+t483-t77.*t327.*2.0+t163.*t319))./2.0)+q_dot_2.*((p_2.*(t430+t448-t73.*t400.*2.0+t159.*t412))./2.0+(p_3.*(t431+t449-t74.*t401.*2.0+t160.*t413))./2.0+(p_4.*(t474+t483-t77.*t447.*2.0+t163.*t452))./2.0)+p_21.*t322+q_ddot_0.*t314+q_ddot_1.*t322+q_ddot_3.*t508+q_ddot_2.*t510-q_dot_3.*((p_2.*(t442-t73.*t406.*2.0-t93.*t392.*2.0+t151.*t388))./2.0+(p_3.*(t443-t74.*t407.*2.0-t94.*t393.*2.0+t152.*t389))./2.0+(p_4.*(t481-t97.*t428.*2.0-t77.*t450.*2.0+t155.*t426))./2.0)-(p_2.*(t316.*t406.*2.0-t324.*t420.*2.0))./2.0;
et15 = p_3.*(t317.*t407.*2.0-t325.*t421.*2.0).*(-1.0./2.0)-(p_4.*(t319.*t450.*2.0-t327.*t455.*2.0))./2.0+q_ddot_4.*(p_7+p_8+t28+(p_2.*(t13.*t59.*2.0+t13.*t69.*2.0))./2.0+(p_3.*(t14.*t59.*2.0+t14.*t69.*2.0))./2.0+(p_4.*(t15.*t59.*2.0+t15.*t69.*2.0))./2.0);
et16 = q_ddot_5.*(p_6+p_7+p_8+p_9+p_10+t28+(p_1.*(t12.*t18.^2.*2.0+t12.*t23.^2.*2.0))./2.0+(p_3.*(t192.^2.*2.0+t196.^2.*2.0))./2.0+(p_2.*(t193.^2.*2.0+t197.^2.*2.0))./2.0+(p_4.*(t248.^2.*2.0+t254.^2.*2.0))./2.0)+q_ddot_0.*t495+q_ddot_1.*t497+q_ddot_6.*t509+q_ddot_2.*t515+(p_1.*(t201.*t278.*2.0-t199.*t312.*2.0))./2.0+(p_2.*(t390.*t410.*2.0-t394.*t416.*2.0))./2.0+(p_3.*(t391.*t411.*2.0-t395.*t417.*2.0))./2.0+(p_4.*(t427.*t451.*2.0-t429.*t454.*2.0))./2.0;
et17 = q_dot_6.*((p_2.*(t440-t95.*t410.*2.0+t197.*t318.*2.0-t193.*t326.*2.0))./2.0+(p_3.*(t441-t96.*t411.*2.0+t196.*t320.*2.0-t192.*t328.*2.0))./2.0-(p_4.*(t475+t482-t254.*t321.*2.0+t248.*t329.*2.0))./2.0)+p_21.*(t84+t88+t90+t91+t92+t323)-q_dot_5.*((p_4.*(t498-t502+t248.*t427.*2.0-t254.*t429.*2.0))./2.0+(p_1.*(t364-t40.*t199.*2.0+t52.*t201-t32.*t312.*2.0))./2.0+(p_2.*(t460+t472+t193.*t390.*2.0-t197.*t394.*2.0))./2.0+(p_3.*(t461+t473+t192.*t391.*2.0-t196.*t395.*2.0))./2.0);
et18 = -q_dot_2.*((p_4.*(t498-t502-t254.*t444.*2.0+t248.*t453.*2.0))./2.0+(p_1.*(t364-t40.*t268.*2.0-t32.*t312.*2.0+t52.*t309))./2.0+(p_2.*(t460+t472-t197.*t398.*2.0+t193.*t414.*2.0))./2.0+(p_3.*(t461+t473-t196.*t399.*2.0+t192.*t415.*2.0))./2.0);
et19 = p_21.*t323+q_ddot_0.*t315+q_ddot_1.*t323+q_ddot_2.*t511+q_ddot_5.*t509+q_dot_6.*((p_2.*(t440-t75.*t326.*2.0+t161.*t318-t95.*t410.*2.0))./2.0+(p_3.*(t441-t76.*t328.*2.0+t162.*t320-t96.*t411.*2.0))./2.0+(p_4.*(t480-t78.*t329.*2.0+t164.*t321-t98.*t451.*2.0))./2.0)+q_dot_2.*((p_2.*(t440-t75.*t414.*2.0-t95.*t410.*2.0+t161.*t398))./2.0+(p_3.*(t441-t76.*t415.*2.0-t96.*t411.*2.0+t162.*t399))./2.0+(p_4.*(t480-t78.*t453.*2.0-t98.*t451.*2.0+t164.*t444))./2.0)-(p_2.*(t318.*t416.*2.0-t326.*t410.*2.0))./2.0-(p_3.*(t320.*t417.*2.0-t328.*t411.*2.0))./2.0-(p_4.*(t321.*t454.*2.0-t329.*t451.*2.0))./2.0;
et20 = q_ddot_6.*(p_7+p_8+t28+(p_2.*(t13.*t60.*2.0+t13.*t70.*2.0))./2.0+(p_3.*(t14.*t60.*2.0+t14.*t70.*2.0))./2.0+(p_4.*(t15.*t60.*2.0+t15.*t70.*2.0))./2.0)-q_dot_5.*((p_2.*(t432-t95.*t394.*2.0-t75.*t416.*2.0+t153.*t390))./2.0+(p_3.*(t433-t96.*t395.*2.0-t76.*t417.*2.0+t154.*t391))./2.0+(p_4.*(t475+t482-t98.*t429.*2.0+t156.*t427))./2.0);
InvDynamics = [et1+et2;et3+et4;et7+et8+et9+et10;et11+et12+et13;et14+et15;et16+et17+et18;et19+et20];
end
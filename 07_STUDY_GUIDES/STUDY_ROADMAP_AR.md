# دليل دراسة المحفظة

هذه مشاريع نُفذت بمساعدة كبيرة من أدوات الذكاء الاصطناعي المحلية، والغرض من الدراسة
فهم الكود والرياضيات والاختبارات وإعادة التجارب بنفسك، لا حفظ وصف جاهز للمقابلة.
حتى مراجعة 9 نوفمبر 2026، اجتازت عشرة مجالات معايير البرمجيات والمحاكاة المحددة.
يبقى D وI نموذجين بحثيين لم يجتازا أهداف الدقة. H اجتاز المعايير بإعداد v2 المحدد؛
فشل v1 التاريخي محفوظ. اقرأ [النتائج والحدود الحالية](../docs/VALIDATION.md).

## ترتيب مقترح

1. A ثم B: العدّ والتزامن وUART وAXI. اشرح reset/enable وrollover وقراءة 48 بت المتسقة.
2. E ثم F: اشتقاق عدم اليقين، اختبار Monte Carlo، وتفسير التوقيت دون ادعاء دقة عتاد.
3. G ثم H: الحركة وEKF وA* وPure Pursuit، ثم TF وSLAM وNav2 داخل المحاكاة.
4. D ثم I: منع تسرب البيانات، المقاييس، المعايرة والمحاذاة الزمنية، وتحليل أسباب الفشل.
5. J ثم K: التخطيط والتحكم، وبعدهما MQTT وتخزين الرسائل واختبارات التأخير والفقد.
6. C ثم L: اربط التوقيت بالروبوت، ثم افهم مهمة UAV وإعادة البناء من العمق وتقييم التغطية.

لكل مشروع: اقرأ المصدر، شغّل اختباراً، أدخل خطأ مقصوداً، ثم فسّر سبب اكتشافه.
الأدلة السابقة في الأدلة التفصيلية تصف تجارب محلية محفوظة؛ النتائج العامة الحالية
في صفحة التحقق، والاختبارات تولّد سجلات جديدة. لا تمثل مراجعة AI اعتماداً مستقلاً.

## الأدلة التفصيلية

- [A_VHDL_AR](A_VHDL_AR.md)
- [B_AXI_LINUX_AR](B_AXI_LINUX_AR.md)
- [C_COSIM_AR](C_COSIM_AR.md)
- [D_Industrial_Inspection_AR](D_Industrial_Inspection_AR.md)
- [E_Uncertainty_AR](E_Uncertainty_AR.md)
- [F_PTP_Timing_AR](F_PTP_Timing_AR.md)
- [G_ROBOT_CORE_AR](G_ROBOT_CORE_AR.md)
- [H_ROS2_NAVIGATION_AR](H_ROS2_NAVIGATION_AR.md)
- [I_VISUAL_INERTIAL_AR](I_VISUAL_INERTIAL_AR.md)
- [J_ROBOT_ARM_AR](J_ROBOT_ARM_AR.md)
- [K_FLEET_TELEMETRY_AR](K_FLEET_TELEMETRY_AR.md)
- [L_UAV_INSPECTION_AR](L_UAV_INSPECTION_AR.md)

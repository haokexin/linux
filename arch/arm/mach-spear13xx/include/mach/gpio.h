/*
 * arch/arm/mach-spear13xx/include/mach/gpio.h
 *
 * GPIO macros for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GPIO_H
#define __MACH_GPIO_H

#ifdef CONFIG_MACH_SPEAR1340_EVB
#include <linux/mfd/stmpe.h>
#endif

/**
 * In Arch SPEAr13xx, plgpio gpio pins are grouped in number of four for
 * interrupt registers. Every group of four gpio's is represented by one bit
 * in IE and MIS registers. This routine returns same irq number for every pin
 * belonging to same group.
 * For arm gpio pins, this routine directly calls __gpio_to_irq routine.
 * It returns the number of the IRQ signaled by this (input) GPIO,
 * or a negative errno.
 */
#define PLGPIO_GROUP_SIZE	4

#define GPIO0_0			0
#define GPIO0_1			1
#define GPIO0_2			2
#define GPIO0_3			3
#define GPIO0_4			4
#define GPIO0_5			5
#define GPIO0_6			6
#define GPIO0_7			7

#define GPIO1_0			8
#define GPIO1_1			9
#define GPIO1_2			10
#define GPIO1_3			11
#define GPIO1_4			12
#define GPIO1_5			13
#define GPIO1_6			14
#define GPIO1_7			15

#if defined(CONFIG_CPU_SPEAR1310_REVA) || defined(CONFIG_CPU_SPEAR1310) || \
		defined(CONFIG_CPU_SPEAR1340)
#define PLGPIO_0		16
#define PLGPIO_1		17
#define PLGPIO_2		18
#define PLGPIO_3		19
#define PLGPIO_4		20
#define PLGPIO_5		21
#define PLGPIO_6		22
#define PLGPIO_7		23
#define PLGPIO_8		24
#define PLGPIO_9		25
#define PLGPIO_10		26
#define PLGPIO_11		27
#define PLGPIO_12		28
#define PLGPIO_13		29
#define PLGPIO_14		30
#define PLGPIO_15		31
#define PLGPIO_16		32
#define PLGPIO_17		33
#define PLGPIO_18		34
#define PLGPIO_19		35
#define PLGPIO_20		36
#define PLGPIO_21		37
#define PLGPIO_22		38
#define PLGPIO_23		39
#define PLGPIO_24		40
#define PLGPIO_25		41
#define PLGPIO_26		42
#define PLGPIO_27		43
#define PLGPIO_28		44
#define PLGPIO_29		45
#define PLGPIO_30		46
#define PLGPIO_31		47
#define PLGPIO_32		48
#define PLGPIO_33		49
#define PLGPIO_34		50
#define PLGPIO_35		51
#define PLGPIO_36		52
#define PLGPIO_37		53
#define PLGPIO_38		54
#define PLGPIO_39		55
#define PLGPIO_40		56
#define PLGPIO_41		57
#define PLGPIO_42		58
#define PLGPIO_43		59
#define PLGPIO_44		60
#define PLGPIO_45		61
#define PLGPIO_46		62
#define PLGPIO_47		63
#define PLGPIO_48		64
#define PLGPIO_49		65
#define PLGPIO_50		66
#define PLGPIO_51		67
#define PLGPIO_52		68
#define PLGPIO_53		69
#define PLGPIO_54		70
#define PLGPIO_55		71
#define PLGPIO_56		72
#define PLGPIO_57		73
#define PLGPIO_58		74
#define PLGPIO_59		75
#define PLGPIO_60		76
#define PLGPIO_61		77
#define PLGPIO_62		78
#define PLGPIO_63		79
#define PLGPIO_64		80
#define PLGPIO_65		81
#define PLGPIO_66		82
#define PLGPIO_67		83
#define PLGPIO_68		84
#define PLGPIO_69		85
#define PLGPIO_70		86
#define PLGPIO_71		87
#define PLGPIO_72		88
#define PLGPIO_73		89
#define PLGPIO_74		90
#define PLGPIO_75		91
#define PLGPIO_76		92
#define PLGPIO_77		93
#define PLGPIO_78		94
#define PLGPIO_79		95
#define PLGPIO_80		96
#define PLGPIO_81		97
#define PLGPIO_82		98
#define PLGPIO_83		99
#define PLGPIO_84		100
#define PLGPIO_85		101
#define PLGPIO_86		102
#define PLGPIO_87		103
#define PLGPIO_88		104
#define PLGPIO_89		105
#define PLGPIO_90		106
#define PLGPIO_91		107
#define PLGPIO_92		108
#define PLGPIO_93		109
#define PLGPIO_94		110
#define PLGPIO_95		111
#define PLGPIO_96		112
#define PLGPIO_97		113
#define PLGPIO_98		114
#define PLGPIO_99		115
#define PLGPIO_100		116
#define PLGPIO_101		117
#define PLGPIO_102		118
#define PLGPIO_103		119
#define PLGPIO_104		120
#define PLGPIO_105		121
#define PLGPIO_106		122
#define PLGPIO_107		123
#define PLGPIO_108		124
#define PLGPIO_109		125
#define PLGPIO_110		126
#define PLGPIO_111		127
#define PLGPIO_112		128
#define PLGPIO_113		129
#define PLGPIO_114		130
#define PLGPIO_115		131
#define PLGPIO_116		132
#define PLGPIO_117		133
#define PLGPIO_118		134
#define PLGPIO_119		135
#define PLGPIO_120		136
#define PLGPIO_121		137
#define PLGPIO_122		138
#define PLGPIO_123		139
#define PLGPIO_124		140
#define PLGPIO_125		141
#define PLGPIO_126		142
#define PLGPIO_127		143
#define PLGPIO_128		144
#define PLGPIO_129		145
#define PLGPIO_130		146
#define PLGPIO_131		147
#define PLGPIO_132		148
#define PLGPIO_133		149
#define PLGPIO_134		150
#define PLGPIO_135		151
#define PLGPIO_136		152
#define PLGPIO_137		153
#define PLGPIO_138		154
#define PLGPIO_139		155
#define PLGPIO_140		156
#define PLGPIO_141		157
#define PLGPIO_142		158
#define PLGPIO_143		159
#define PLGPIO_144		160
#define PLGPIO_145		161
#define PLGPIO_146		162
#define PLGPIO_147		163
#define PLGPIO_148		164
#define PLGPIO_149		165
#define PLGPIO_150		166
#define PLGPIO_151		167
#define PLGPIO_152		168
#define PLGPIO_153		169
#define PLGPIO_154		170
#define PLGPIO_155		171
#define PLGPIO_156		172
#define PLGPIO_157		173
#define PLGPIO_158		174
#define PLGPIO_159		175
#define PLGPIO_160		176
#define PLGPIO_161		177
#define PLGPIO_162		178
#define PLGPIO_163		179
#define PLGPIO_164		180
#define PLGPIO_165		181
#define PLGPIO_166		182
#define PLGPIO_167		183
#define PLGPIO_168		184
#define PLGPIO_169		185
#define PLGPIO_170		186
#define PLGPIO_171		187
#define PLGPIO_172		188
#define PLGPIO_173		189
#define PLGPIO_174		190
#define PLGPIO_175		191
#define PLGPIO_176		192
#define PLGPIO_177		193
#define PLGPIO_178		194
#define PLGPIO_179		195
#define PLGPIO_180		196
#define PLGPIO_181		197
#define PLGPIO_182		198
#define PLGPIO_183		199
#define PLGPIO_184		200
#define PLGPIO_185		201
#define PLGPIO_186		202
#define PLGPIO_187		203
#define PLGPIO_188		204
#define PLGPIO_189		205
#define PLGPIO_190		206
#define PLGPIO_191		207
#define PLGPIO_192		208
#define PLGPIO_193		209
#define PLGPIO_194		210
#define PLGPIO_195		211
#define PLGPIO_196		212
#define PLGPIO_197		213
#define PLGPIO_198		214
#define PLGPIO_199		215
#define PLGPIO_200		216
#define PLGPIO_201		217
#define PLGPIO_202		218
#define PLGPIO_203		219
#define PLGPIO_204		220
#define PLGPIO_205		221
#define PLGPIO_206		222
#define PLGPIO_207		223
#define PLGPIO_208		224
#define PLGPIO_209		225
#define PLGPIO_210		226
#define PLGPIO_211		227
#define PLGPIO_212		228
#define PLGPIO_213		229
#define PLGPIO_214		230
#define PLGPIO_215		231
#define PLGPIO_216		232
#define PLGPIO_217		233
#define PLGPIO_218		234
#define PLGPIO_219		235
#define PLGPIO_220		236
#define PLGPIO_221		237
#define PLGPIO_222		238
#define PLGPIO_223		239
#define PLGPIO_224		240
#define PLGPIO_225		241
#define PLGPIO_226		242
#define PLGPIO_227		243
#define PLGPIO_228		244
#define PLGPIO_229		245
#define PLGPIO_230		246
#define PLGPIO_231		247
#define PLGPIO_232		248
#define PLGPIO_233		249
#define PLGPIO_234		250
#define PLGPIO_235		251
#define PLGPIO_236		252
#define PLGPIO_237		253
#define PLGPIO_238		254
#define PLGPIO_239		255
#define PLGPIO_240		256
#define PLGPIO_241		257
#define PLGPIO_242		258
#define PLGPIO_243		259
#define PLGPIO_244		260
#define PLGPIO_245		261
#define PLGPIO_246		262
#define PLGPIO_247		263
#define PLGPIO_248		264
#define PLGPIO_249		265

#endif /* CPU_SPEAR1310_REVA, CPU_SPEAR1310, CPU_SPEAR1340 */

#ifdef CONFIG_MACH_SPEAR1340_EVB
#define SPEAR_STMPE801_GPIO_BASE	(PLGPIO_249 + 1)
#define SPEAR_STMPE801_GPIO_END	(SPEAR_STMPE801_GPIO_BASE + STMPE_NR_GPIOS - 1)
#define STMPE801_GPIO_0		(SPEAR_STMPE801_GPIO_BASE + 0)
#define STMPE801_GPIO_1		(SPEAR_STMPE801_GPIO_BASE + 1)
#define STMPE801_GPIO_2		(SPEAR_STMPE801_GPIO_BASE + 2)
#define STMPE801_GPIO_3		(SPEAR_STMPE801_GPIO_BASE + 3)
#define STMPE801_GPIO_4		(SPEAR_STMPE801_GPIO_BASE + 4)
#define STMPE801_GPIO_5		(SPEAR_STMPE801_GPIO_BASE + 5)
#define STMPE801_GPIO_6		(SPEAR_STMPE801_GPIO_BASE + 6)
#define STMPE801_GPIO_7		(SPEAR_STMPE801_GPIO_BASE + 7)
#define ARCH_NR_GPIOS		(SPEAR_STMPE801_GPIO_END + 1)
#else
/* spear 13xx have 266 gpio pins */
#define ARCH_NR_GPIOS		(PLGPIO_249 + 1)
#endif

#include <plat/gpio.h>

#endif /* __MACH_GPIO_H */

#ifndef __DRV2605_REG_H_
#define __DRV2605_REG_H_


#define DRV2605_REG_Status 0x00
#define DRV2605_REG_Status_Bit_DeviceID 0x05 //3bits,DRV2605L=0x07(111)
#define DRV2605_REG_Status_Bit_DiagnosticsResult 0x03 //auto_cali/self_diag,1bit,0=OK,1=Error
#define DRV2605_REG_Status_Bit_OverTemperature 0x01 //1bit,0=OK,1=Error
#define DRV2605_REG_Status_Bit_OverCurrent 0x00 //1bit,0=OK,1=Error

#define DRV2605_REG_Mode 0x01
#define DRV2605_REG_Mode_Bit_Reset 0x07 //1bit,0=Normal,1=Reset
#define DRV2605_REG_Mode_Bit_Standby 0x06 //1bit,0=Active,1=software Standby,DEFAULT=1
#define DRV2605_REG_Mode_Bit_Standby_EN 0
#define DRV2605_REG_Mode_Bit_Standby_DE 1
#define DRV2605_REG_Mode_Bit_Mode 0x00 
#define DRV2605_REG_Mode_Bit_Mode_Int_trig 0x00
#define DRV2605_REG_Mode_Bit_Mode_Ext_edge 0x01
#define DRV2605_REG_Mode_Bit_Mode_Ext_level 0x02
#define DRV2605_REG_Mode_Bit_Mode_PWM_Analog 0x03
#define DRV2605_REG_Mode_Bit_Mode_Audio2Vibe 0x04
#define DRV2605_REG_Mode_Bit_Mode_RTP 0x05
#define DRV2605_REG_Mode_Bit_Mode_Diagnostics 0x06
#define DRV2605_REG_Mode_Bit_Mode_AutoCali 0x07

/*
3bits,DEFAULT=0x00
0=Internal trig,
1=External trig(edge detect),
2=External trig(level detect),
3=PWM&Analog Input,
4=Audio_to_vibe,
5=RTP mode,
6=Diagnostics,
7=Auto calibration
*/

#define DRV2605_REG_RTPInput 0x02
#define DRV2605_REG_RTPInput_Bit_RTPData 0x00 //8bits,0x00~0xFF

#define DRV2605_REG_LibSel 0x03
#define DRV2605_REG_LibSel_Bit_HI_Z 0x04 //1bit,0=Normal,1=Hi-Z
#define DRV2605_REG_LibSel_Bit_Lib 0x00 
#define DRV2605_REG_LibSel_Bit_Lib_ERM_A 0x01
#define DRV2605_REG_LibSel_Bit_Lib_ERM_B 0x02
#define DRV2605_REG_LibSel_Bit_Lib_ERM_C 0x03
#define DRV2605_REG_LibSel_Bit_Lib_ERM_D 0x04
#define DRV2605_REG_LibSel_Bit_Lib_ERM_E 0x05
#define DRV2605_REG_LibSel_Bit_Lib_LRA   0x06
#define DRV2605_REG_LibSel_Bit_Lib_ERM_F 0x07
/*
3bits,DEFAULT=0x01
0=Empty,
1-5+7=ERM Lib A-F
6=LRA Lib
*/

#define DRV2605_REG_WaveformSeq_1 0x04
#define DRV2605_REG_WaveformSeq_2 0x05
#define DRV2605_REG_WaveformSeq_3 0x06
#define DRV2605_REG_WaveformSeq_4 0x07
#define DRV2605_REG_WaveformSeq_5 0x08
#define DRV2605_REG_WaveformSeq_6 0x09
#define DRV2605_REG_WaveformSeq_7 0x0A
#define DRV2605_REG_WaveformSeq_8 0x0B
#define DRV2605_REG_WaveformSeq_Bit_wait 0x07 //1bit,0=No wait,1=enable Wait
#define DRV2605_REG_WaveformSeq_Bit_WavSel 0x00 //7bits,1-123

#define DRV2605_REG_Go 0x0C
#define DRV2605_REG_Go_Bit_Go 0x00 //1bit,0=No effect,1=Start,clear after sequence all complete or run into empty sequence

#define DRV2605_REG_OverdriveTimeOffset 0x0D
#define DRV2605_REG_OverdriveTimeOffset_Bit_ODT 0x00 

#define DRV2605_REG_SustainTimeOffset_Pos 0x0E
#define DRV2605_REG_SustainTimeOffset_Pos_Bit_SPT 0x00

#define DRV2605_REG_SustainTimeOffset_Neg 0x0F
#define DRV2605_REG_SustainTimeOffset_Neg_Bit_SNT 0x00

#define DRV2605_REG_BrakeTimeOffset 0x10
#define DRV2605_REG_BrakeTimeOffset_Bit_BRT 0x00

#define DRV2605_REG_AudioToVibeCtrl 0x11
#define DRV2605_REG_AudioToVibeCtrl_Bit_Peaktime 0x02 //2bits
#define DRV2605_REG_AudioToVibeCtrl_Bit_Filt 0x00 //2bits

#define DRV2605_REG_AudioToVibeMinInput 0x12
#define DRV2605_REG_AudioToVibeMinInput_Bit_AMIN 0x00 //8bits

#define DRV2605_REG_AudioToVibeMaxInput 0x13
#define DRV2605_REG_AudioToVibeMaxInput_Bit_AMAX 0x00 //8bits

#define DRV2605_REG_AudioToVibeMinOutput 0x14
#define DRV2605_REG_AudioToVibeMinOutput_Bit_AOUTMIN 0x00 //8bits

#define DRV2605_REG_AudioToVibeMaxOutput 0x15
#define DRV2605_REG_AudioToVibeMaxOutput_Bit_AOUTMAX 0x00 //8bits

#define DRV2605_REG_RatedVoltage 0x16
#define DRV2605_REG_RatedVoltage_Bit_RATEDV 0x00 //8bits

#define DRV2605_REG_ODClamp 0x17
#define DRV2605_REG_ODClamp_Bit_ODC 0x00

#define DRV2605_REG_AutoCaliCompResult 0x18
#define DRV2605_REG_AutoCaliCompResult_Bit_ACR 0x00

#define DRV2605_REG_AutoCaliBackEMFResult 0x19
#define DRV2605_REG_AutoCaliBackEMFResult_Bit_BEMF 0x00

#define DRV2605_REG_FeedbackCtrl 0x1A
#define DRV2605_REG_FeedbackCtrl_Bit_Sel 0x07 //1bit,0=ERM,1=LRA
#define DRV2605_REG_FeedbackCtrl_Bit_Sel_LRA 0x01
#define DRV2605_REG_FeedbackCtrl_Bit_Sel_ERM 0x00
#define DRV2605_REG_FeedbackCtrl_Bit_FB_BrakeFactor 0x04 //3bits
#define DRV2605_REG_FeedbackCtrl_Bit_Loop_Gain 0x02 //2bits
#define DRV2605_REG_FeedbackCtrl_Bit_BEMF_Gain 0x00 //2bits

#define DRV2605_REG_VBAT_Monitor 0x21
#define DRV2605_REG_VBAT_Monitor_Bit_VBAT 0x00 //8bits(5.6VxVBAT/255)
//Waveform Lib
#define DRV2605_WAV_1 0x01
#define DRV2605_WAV_2 0x02
#define DRV2605_WAV_3 0x03
#define DRV2605_WAV_4 0x04
#define DRV2605_WAV_5 0x05
#define DRV2605_WAV_6 0x06
#define DRV2605_WAV_7 0x07
#define DRV2605_WAV_8 0x08
#define DRV2605_WAV_9 0x09
#define DRV2605_WAV_10 0x0A
#define DRV2605_WAV_11 0x0B
#define DRV2605_WAV_12 0x0C
#define DRV2605_WAV_13 0x0D
#define DRV2605_WAV_14 0x0E
#define DRV2605_WAV_15 0x0F
#define DRV2605_WAV_16 0x10
#define DRV2605_WAV_17 0x11
#define DRV2605_WAV_18 0x12
#define DRV2605_WAV_19 0x13
#define DRV2605_WAV_20 0x14
#define DRV2605_WAV_21 0x15
#define DRV2605_WAV_22 0x16
#define DRV2605_WAV_23 0x17
#define DRV2605_WAV_24 0x18
#define DRV2605_WAV_25 0x19
#define DRV2605_WAV_26 0x1A
#define DRV2605_WAV_27 0x1B
#define DRV2605_WAV_28 0x1C
#define DRV2605_WAV_29 0x1D
#define DRV2605_WAV_30 0x1E
#define DRV2605_WAV_31 0x1F
#define DRV2605_WAV_32 0x20
#define DRV2605_WAV_33 0x21
#define DRV2605_WAV_34 0x22
#define DRV2605_WAV_35 0x23
#define DRV2605_WAV_36 0x24
#define DRV2605_WAV_37 0x25
#define DRV2605_WAV_38 0x26
#define DRV2605_WAV_39 0x27
#define DRV2605_WAV_40 0x28
#define DRV2605_WAV_41 0x29
#define DRV2605_WAV_42 0x2A
#define DRV2605_WAV_43 0x2B
#define DRV2605_WAV_44 0x2C
#define DRV2605_WAV_45 0x2D
#define DRV2605_WAV_46 0x2E
#define DRV2605_WAV_47 0x2F
#define DRV2605_WAV_48 0x30
#define DRV2605_WAV_49 0x31
#define DRV2605_WAV_50 0x32
#define DRV2605_WAV_51 0x33
#define DRV2605_WAV_52 0x34
#define DRV2605_WAV_53 0x35
#define DRV2605_WAV_54 0x36
#define DRV2605_WAV_55 0x37
#define DRV2605_WAV_56 0x38
#define DRV2605_WAV_57 0x39
#define DRV2605_WAV_58 0x3A
#define DRV2605_WAV_59 0x3B
#define DRV2605_WAV_60 0x3C
#define DRV2605_WAV_61 0x3D
#define DRV2605_WAV_62 0x3E
#define DRV2605_WAV_63 0x3F
#define DRV2605_WAV_64 0x40
#define DRV2605_WAV_65 0x41
#define DRV2605_WAV_66 0x42
#define DRV2605_WAV_67 0x43
#define DRV2605_WAV_68 0x44
#define DRV2605_WAV_69 0x45
#define DRV2605_WAV_70 0x46
#define DRV2605_WAV_71 0x47
#define DRV2605_WAV_72 0x48
#define DRV2605_WAV_73 0x49
#define DRV2605_WAV_74 0x4A
#define DRV2605_WAV_75 0x4B
#define DRV2605_WAV_76 0x4C
#define DRV2605_WAV_77 0x4D
#define DRV2605_WAV_78 0x4E
#define DRV2605_WAV_79 0x4F
#define DRV2605_WAV_80 0x50
#define DRV2605_WAV_81 0x51
#define DRV2605_WAV_82 0x52
#define DRV2605_WAV_83 0x53
#define DRV2605_WAV_84 0x54
#define DRV2605_WAV_85 0x55
#define DRV2605_WAV_86 0x56
#define DRV2605_WAV_87 0x57
#define DRV2605_WAV_88 0x58
#define DRV2605_WAV_89 0x59
#define DRV2605_WAV_90 0x5A
#define DRV2605_WAV_91 0x5B
#define DRV2605_WAV_92 0x5C
#define DRV2605_WAV_93 0x5D
#define DRV2605_WAV_94 0x5E
#define DRV2605_WAV_95 0x5F
#define DRV2605_WAV_96 0x60
#define DRV2605_WAV_97 0x61
#define DRV2605_WAV_98 0x62
#define DRV2605_WAV_99 0x63
#define DRV2605_WAV_100 0x64
#define DRV2605_WAV_101 0x65
#define DRV2605_WAV_102 0x66
#define DRV2605_WAV_103 0x67
#define DRV2605_WAV_104 0x68
#define DRV2605_WAV_105 0x69
#define DRV2605_WAV_106 0x6A
#define DRV2605_WAV_107 0x6B
#define DRV2605_WAV_108 0x6C
#define DRV2605_WAV_109 0x6D
#define DRV2605_WAV_110 0x6E
#define DRV2605_WAV_111 0x6F
#define DRV2605_WAV_112 0x70
#define DRV2605_WAV_113 0x71
#define DRV2605_WAV_114 0x72
#define DRV2605_WAV_115 0x73
#define DRV2605_WAV_116 0x74
#define DRV2605_WAV_117 0x75
#define DRV2605_WAV_118 0x76
#define DRV2605_WAV_119 0x77
#define DRV2605_WAV_120 0x78
#define DRV2605_WAV_121 0x79
#define DRV2605_WAV_122 0x7A
#define DRV2605_WAV_123 0x7B


#endif //__DRV2605_REG_H_
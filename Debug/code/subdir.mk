################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/display_tft180.c" \
"../code/element_recognition.c" \
"../code/motor_control.c" \
"../code/pid_control.c" \
"../code/show_speed.c" \
"../code/smart_car.c" \
"../code/vision_track.c" 

COMPILED_SRCS += \
"code/display_tft180.src" \
"code/element_recognition.src" \
"code/motor_control.src" \
"code/pid_control.src" \
"code/show_speed.src" \
"code/smart_car.src" \
"code/vision_track.src" 

C_DEPS += \
"./code/display_tft180.d" \
"./code/element_recognition.d" \
"./code/motor_control.d" \
"./code/pid_control.d" \
"./code/show_speed.d" \
"./code/smart_car.d" \
"./code/vision_track.d" 

OBJS += \
"code/display_tft180.o" \
"code/element_recognition.o" \
"code/motor_control.o" \
"code/pid_control.o" \
"code/show_speed.o" \
"code/smart_car.o" \
"code/vision_track.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/display_tft180.src":"../code/display_tft180.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/display_tft180.o":"code/display_tft180.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/element_recognition.src":"../code/element_recognition.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/element_recognition.o":"code/element_recognition.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/motor_control.src":"../code/motor_control.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/motor_control.o":"code/motor_control.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/pid_control.src":"../code/pid_control.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/pid_control.o":"code/pid_control.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/show_speed.src":"../code/show_speed.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/show_speed.o":"code/show_speed.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/smart_car.src":"../code/smart_car.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/smart_car.o":"code/smart_car.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/vision_track.src":"../code/vision_track.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/admin/AURIX-v1.10.24-workspace/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/vision_track.o":"code/vision_track.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/display_tft180.d ./code/display_tft180.o ./code/display_tft180.src ./code/element_recognition.d ./code/element_recognition.o ./code/element_recognition.src ./code/motor_control.d ./code/motor_control.o ./code/motor_control.src ./code/pid_control.d ./code/pid_control.o ./code/pid_control.src ./code/show_speed.d ./code/show_speed.o ./code/show_speed.src ./code/smart_car.d ./code/smart_car.o ./code/smart_car.src ./code/vision_track.d ./code/vision_track.o ./code/vision_track.src

.PHONY: clean-code


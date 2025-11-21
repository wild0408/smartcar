################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/display_tft180.c \
../code/element_recognition.c \
../code/motor_control.c \
../code/path_planning.c \
../code/pid_control.c \
../code/position_control.c \
../code/show_speed.c \
../code/smart_car.c \
../code/vision_track.c 

COMPILED_SRCS += \
./code/display_tft180.src \
./code/element_recognition.src \
./code/motor_control.src \
./code/path_planning.src \
./code/pid_control.src \
./code/position_control.src \
./code/show_speed.src \
./code/smart_car.src \
./code/vision_track.src 

C_DEPS += \
./code/display_tft180.d \
./code/element_recognition.d \
./code/motor_control.d \
./code/path_planning.d \
./code/pid_control.d \
./code/position_control.d \
./code/show_speed.d \
./code/smart_car.d \
./code/vision_track.d 

OBJS += \
./code/display_tft180.o \
./code/element_recognition.o \
./code/motor_control.o \
./code/path_planning.o \
./code/pid_control.o \
./code/position_control.o \
./code/show_speed.o \
./code/smart_car.o \
./code/vision_track.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/car/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/display_tft180.d ./code/display_tft180.o ./code/display_tft180.src ./code/element_recognition.d ./code/element_recognition.o ./code/element_recognition.src ./code/motor_control.d ./code/motor_control.o ./code/motor_control.src ./code/path_planning.d ./code/path_planning.o ./code/path_planning.src ./code/pid_control.d ./code/pid_control.o ./code/pid_control.src ./code/position_control.d ./code/position_control.o ./code/position_control.src ./code/show_speed.d ./code/show_speed.o ./code/show_speed.src ./code/smart_car.d ./code/smart_car.o ./code/smart_car.src ./code/vision_track.d ./code/vision_track.o ./code/vision_track.src

.PHONY: clean-code


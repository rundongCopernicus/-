################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/car.c \
../code/falsh.c \
../code/fuction.c \
../code/gps.c \
../code/imu.c \
../code/kalman.c \
../code/motor.c \
../code/navigation.c \
../code/pid.c \
../code/remote.c 

OBJS += \
./code/car.o \
./code/falsh.o \
./code/fuction.o \
./code/gps.o \
./code/imu.o \
./code/kalman.o \
./code/motor.o \
./code/navigation.o \
./code/pid.o \
./code/remote.o 

COMPILED_SRCS += \
./code/car.src \
./code/falsh.src \
./code/fuction.src \
./code/gps.src \
./code/imu.src \
./code/kalman.src \
./code/motor.src \
./code/navigation.src \
./code/pid.src \
./code/remote.src 

C_DEPS += \
./code/car.d \
./code/falsh.d \
./code/fuction.d \
./code/gps.d \
./code/imu.d \
./code/kalman.d \
./code/motor.d \
./code/navigation.d \
./code/pid.d \
./code/remote.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/32/ads/project/my_code/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '



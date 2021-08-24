#!/bin/sh

EXE=$1
CONFIG=config.props
PARAM=model.props
PROCS=1

valid_option()
{
	# check if command parameter is valid
	if [ "$EXE" == "" ]; then
		echo "USAGE: run.sh <model>"
		exit -1
	fi

	# check if model execution file exists
	if [ ! -f "$EXE" ]; then
		echo "Model(${EXE}) is missing"
		exit -1
	fi

	# check if config.props exists
	if [ ! -f "config.props" ]; then
		echo "config.props is missing"
		exit -1
	fi

	# check if model.props exists
	if [ ! -f "model.props" ]; then
		echo "model.props is missing"
		exit -1
	else
		# define number of MPI processes

		P=`grep proc.mpi model.props | cut -d'=' -f2`
		if [ ! "$EXE" == "" ]; then
			PROCS=${P}
		fi
	fi
}

model_run()
{
	mpirun -n ${PROCS} ${EXE} ${CONFIG} ${PARAM}
}

valid_option
model_run

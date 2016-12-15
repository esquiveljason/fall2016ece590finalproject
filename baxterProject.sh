ROS_PATH=/home/student/ros_ws/

ACH_CHAN_BAXTER_REF='baxterInterface'
ACH_CHAN_BAXTER_STATE='fromaxterInterface'

MakeAch()
{
	ach -1 -C $ACH_CHAN_BAXTER_REF -m 10 -n 3000
	ach -1 -C $ACH_CHAN_BAXTER_STATE -m 10 -n 3000
}

EnableBaxter()
{
	rosrun baxter_tools_enable -e
}

DisableBaxter()
{
	rosrun baxter_tools_enable -d
}

StartBaxter()
{
	roslaunch baxter_gazebo baxter_world.launch
}

StartInterface()
{
	rosrun baxter_examples scripts/ECE590FinalProjectBaxter/src/baxterInterface.py
}

SetWorkSpace()
{
	cd $ROS_PATH
	case $1 in
		'sim')
			./baxter.sh sim
		;;
		'real')
			./baxter.sh
		;;
		*)
			echo 'no baxter type defined'
		;;
	esac
}
Start()
{
	echo '1243'
	SetWorkSpace $1
	echo 'abc'
	StartRobot 
}
Stop()
{	MakeAch
	sleep 20
	EnableBaxter

}
case $1 in
	'workspace')
		SetWorkSpace $2
	;;
	'startBaxter')
		StartBaxter
	;;
	'stop')
		Stop
	;;
	'startInterface')
		EnableBaxter
		StartInterface
	;;
	*)
		echo 'Commands'
		echo '	: workspace - sets up workspace'
		echo ' 		: real - starts real robot'
		echo '		: sim  - starts sim robot'
		echo '	: startBaxter - Starts Robot'
		echo ' 	: stop - removes the file'
	;;
esac

exit 0

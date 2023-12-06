cd $0/..
chmod u+rx auto.sh

printf '请选择启动代码的模式\n1.debug    2.release\n'
while true; do
	read make_mode
	if [ $make_mode = 1 ]; then
		sed -i "s/#set(DEBUG/set(DEBUG/g" ./CMakeLists.txt
		break
	elif [ $make_mode = 2 ]; then
		sed -i "s/set(DEBUG/#set(DEBUG/g" ./CMakeLists.txt
		break
	else
		printf '请输入正确的数字\n'
	fi
done
printf '请选择使用何种摄像头\n1.电脑内置摄像头    2.外接摄像头(默认大恒)\n'
while true; do
	read make_mode
	if [ $make_mode = 1 ]; then
		sed -i "s/#set(USINGVIDEO0/set(USINGVIDEO0/g" ./CMakeLists.txt
		break
	elif [ $make_mode = 2 ]; then
		sed -i "s/set(USINGVIDEO0/#set(USINGVIDEO0/g" ./CMakeLists.txt
		break
	else
		printf '请输入正确的数字\n'
	fi
done

#默认删除build
rm -r ./build

mkdir build && cd build

cmake ..

make -j4

./rm_version4

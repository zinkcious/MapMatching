运行环境python2.7解释器，预装matplotlib包

在工程目录下打开命令窗口，输入：
	python bin/MapMatching.py data/StarTrek.txt data/StarTrek_gps.dat result/output.txt

输出结果就会在result目录的output.txt中，同时会输出道路可视化结果在output.txt所在目录的pic文件夹下

其中：
StarTrek_gps.dat的每一条数据是某一辆汽车的一个gps坐标和朝向， 每一行数据中，从左到右分别为：汽车编号， 时间， 经度，维度，速度，方向
StarTrek.txt 用来表示地图信息，每一条数据是地图上一条道路的信息

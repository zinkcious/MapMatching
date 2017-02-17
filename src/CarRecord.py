class CarRecord:
    def __init__(self, argv):
        self.car_id = int(argv[0])
        self.time = argv[1]
        self.geo = [float(ele) for ele in argv[2:4]]
        self.speed = float(argv[4])
        self.direction = float(argv[5])
with open("/home/kkw0418/Downloads/extractor+matcher/superpoint_max+NN-mutual/outputs/demo/sfm/cameras.bin", "rb") as f:
    data = f.read()
    for i, byte in enumerate(data):
        print(byte.decode("ascii"))
import MapCoodinator

def mapCoodinatorTest():
    # Create a new instance of GoogleMap 
    
    lat = 35.725496
    lng = 140.056374
    gmd = MapCoodinator.MapCoodinator(lat, lng, 19)

    print("The tile coorindates are {}".format(gmd.getXY(lat, lng)))

    try:
        # Get the high resolution image
        img = gmd.generateImage()
    except IOError:
        print("Could not generate the image - try adjusting the zoom level and checking your coordinates")
    else:
        # Save the image to disk
        img.save("high_resolution_image.png")
        print("The map has successfully been created")

    
    point_x, point_y = gmd.getXY(lat, lng)
    point_x %= 256 + 256
    point_y %= 256 + 256
    test_lat, test_lng = gmd.getGPS(point_x, point_y)
    print("diff of lat, log are {0}, {1}".format(test_lat, test_lng))


def main():
    mapCoodinatorTest()

if __name__ == '__main__':  main()
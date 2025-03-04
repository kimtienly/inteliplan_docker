class ColorNames:
    OrionColorMap = {}
    OrionColorMap["white"] = "#FFFFFF"
    OrionColorMap["silver"] = "#C0C0C0"
    OrionColorMap["grey"] = "#808080"
    OrionColorMap["black"] = "#000000"
    OrionColorMap["red"] = "#FF0000"
    OrionColorMap["maroon"] = "#800000"
    OrionColorMap["yellow"] = "#FFFF00"
    OrionColorMap["olive"] = "#808000"
    OrionColorMap["lime"] = "#00FF00"
    OrionColorMap["green"] = "#008000"
    OrionColorMap["aqua"] = "#00FFFF"
    OrionColorMap["teal"] = "#008080"
    OrionColorMap["blue"] = "#0000FF"
    OrionColorMap["navy"] = "#000080"
    OrionColorMap["fuschia"] = "#FF00FF"
    OrionColorMap["purple"] = "#800080"
    OrionColorMap["orange"] = "#F39C12"

    @staticmethod
    def rgbFromStr(s):
        # s starts with a #.
        r, g, b = int(s[1:3], 16), int(s[3:5], 16), int(s[5:7], 16)
        return r, g, b

    @staticmethod
    def findNearestOrionColorName(RGB_tuple):
        return ColorNames.findNearestColorName(RGB_tuple, ColorNames.OrionColorMap)

    @staticmethod
    def findNearestColorName(RGB_tuple, Map):
        R = RGB_tuple[0]
        G = RGB_tuple[1]
        B = RGB_tuple[2]
        mindiff = None
        for d in Map:
            r, g, b = ColorNames.rgbFromStr(Map[d])
            diff = abs(R - r) * 256 + abs(G - g) * 256 + abs(B - b) * 256
            if mindiff is None or diff < mindiff:
                mindiff = diff
                mincolorname = d
        return mincolorname

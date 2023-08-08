import json

class TmJasonToDiction:
    @staticmethod
    def jason_to_dic(jasonString):
        json_array = json.loads(jasonString)
        returnDictionary={}
        for dic in json_array:
            returnDictionary.update({dic['Item'] : dic['Value']})
        return returnDictionary

    @staticmethod
    def tm_string_to_jason(string):

        pos = [pos for pos, char in enumerate(string) if char == ',']

        #newString = string[:-4]
        newString = string[:pos[len(pos)-1]]
        for i in range(4):
            commaPosition = newString.find(',')
            newString = newString[commaPosition+1:]
        return newString

    @staticmethod
    def split_package(string):
        headerPosition = [i for i in range(len(string)) if string.startswith('$TMSVR', i)]

        if(len(headerPosition) == 0):
            return string,None
        starPosition = [i for i in range(len(string)) if string.startswith('*', i)]
        if(len(starPosition) == 0):
            return string,None

        i=0
        newString =[]
        remainString=''
        while(i<len(starPosition)):
            if(i==len(headerPosition)):
                break
            stringStart = headerPosition[i]             
            if(i+1>=len(headerPosition)):
                stringEnd = len(string)
            else:
                if(headerPosition[i+1]<starPosition[i]):
                    headerPosition.remove(headerPosition[i+1])
                    continue
                stringEnd = headerPosition[i+1]
            i+=1
            newString.append(string[stringStart:stringEnd])

        
        if(stringEnd<len(string)):
            remainString = string[stringEnd:]
            return remainString,newString
        if(-1 == newString[len(newString)-1].find('*') ):
            remainString = newString[len(newString)-1]
            del newString[-1]
       
        return remainString,newString

def print_splited_string_and_nokori(newString,nokori):
    if(newString is not None):
        for string in newString:
            print(string)
    print("***nokori is***")
    print(nokori)

if __name__ == "__main__":
    inputStr = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E'
    jasonString = TmJasonToDiction.tm_string_to_jason(inputStr)
    print(jasonString)
    dictionary = TmJasonToDiction.jason_to_dic(jasonString)
    print(dictionary)

#<ok>               in test 4
#<ok><ok>           in test 2
#<ok><ok><nokori>   in test 7
#<nokori>           in test 10
#<error>            in test 8
#<error><ok>        in test 5
#<error><ok><ok>    in test 9
#<error><ok><nokori>in test 6
#<error><nokori>    in test 3

    print("------test 2----<ok><ok><nokori>------")
    inputStr2 = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.37033'

    nokori,newString= TmJasonToDiction.split_package(inputStr2)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 3-----<error><nokori>-----")
    inputStr3 = 'VR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value"'
    nokori,newString= TmJasonToDiction.split_package(inputStr3)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 4----<ok>------")
    inputStr4 = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E'
    nokori,newString= TmJasonToDiction.split_package(inputStr4)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 5----<error><ok>------")
    inputStr5 = 'Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2B $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2B'
    nokori,newString= TmJasonToDiction.split_package(inputStr5)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 6----<error><ok><nokori>------")
    inputStr6 = 'Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2B $TMSVR,228,5,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.'
    nokori,newString= TmJasonToDiction.split_package(inputStr6)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 7----<ok><ok><nokori>------")
    inputStr7 = '$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987'
    nokori,newString= TmJasonToDiction.split_package(inputStr7)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 8----<error>------")
    inputStr8 = '228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Val'
    nokori,newString= TmJasonToDiction.split_package(inputStr8)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 9----<error><ok><ok>------")
    inputStr9 = 'R,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E$TMSVR,228,0,3,[{"Item":"Robot_Link","Value":1},{"Item":"Joint_Angle","Value":[-5.303604,55.2007942,69.6347,-29.8407936,79.27841,6.384208]},{"Item":"Coord_Base_Tool","Value":[1053.882,-277.370331,90.85741,-175.800278,11.7489395,79.05987]}],*2E'
    nokori,newString= TmJasonToDiction.split_package(inputStr9)
    print_splited_string_and_nokori(newString,nokori)

    print("------test 10----<nokori>------")
    inputStr10 = '$TMSVR,228,0,3,[{"Item":'
    nokori,newString= TmJasonToDiction.split_package(inputStr10)
    print_splited_string_and_nokori(newString,nokori)
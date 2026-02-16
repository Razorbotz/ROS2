#pragma once
#include <string>
#include <list>
#include <utility>
#include <memory>
#include <vector>
#include <cstdarg>
#include <string>

enum TYPE {
    OBJECT = 0,
    BOOLEAN = 1,
    CHARACTER = 2,
    INT8 = 3,
    INT16 = 4,
    INT32 = 5,
    INT64 = 6,
    UINT8 = 7,
    UINT16 = 8,
    UINT32 = 9,
    UINT64 = 10,
    FLOAT32 = 11,
    FLOAT64 = 12,
    STRING = 13,
    ARRAYBOOLEAN = 14,
    ARRAYCHARACTER = 15,
    ARRAYINT8 = 16,
    ARRAYINT16 = 17,
    ARRAYINT32 = 18,
    ARRAYINT64 = 19,
    ARRAYUINT8 = 20,
    ARRAYUINT16 = 21,
    ARRAYUINT32 = 22,
    ARRAYUINT64 = 23,
    ARRAYFLOAT32 = 24,
    ARRAYFLOAT64 = 25
};

union Data{
    bool boolean;
    char character;
    int8_t int8;
    int16_t int16;
    int32_t int32;
    int64_t int64;
    uint8_t uint8;
    uint16_t uint16;
    uint32_t uint32;
    uint64_t uint64;
    float float32;
    double float64;
};

enum Field_Strings{
    DeviceID = 1,
    BusVoltage = 2,
    OutputCurrent = 3,
    OutputPercent = 4,
    Temperature = 5,
    SensorPosition = 6,
    SensorVelocity = 7,
    MaxCurrent = 8,
    TempDisable = 9,
    Error = 10,
    Voltage = 11,
    MotorNumber = 12,
    Speed = 13,
    Potentiometer = 14,
    TimeWithoutChange = 15,
    Max = 16,
    Min = 17,
    AtMin = 18,
    AtMax = 19,
    Distance = 20,
    Sensorless = 21,
    RobotState = 22,
    ExcavationState = 23,
    ErrorState = 24,
    DiagnosticsState = 25,
    TiltState = 26,
    DumpState = 27,
    LevelBucket = 28,
    LevelArms = 29,
    DestX = 30,
    DestZ = 31,
    X = 32,
    Y = 33,
    Z = 34,
    Roll = 35,
    Pitch = 36,
    Yaw = 37,
    Aruco = 38,
    RSSI = 39,
    WiFi = 40,
    CANBus = 41,
    UsingCan1 = 42,
    RXPackets = 43,
    TXPackets = 44,
    CANBus2 = 45,
    RX2Packets = 46,
    TX2Packets = 47,
    FirstMotor = 48,
    SecondMotor = 49,
    NumBreaks = 50,
    F1Vel = 51,
    F1RPM = 52,
    F1Speed = 53,
    F2Vel = 54,
    F2RPM = 55,
    F2Speed = 56,
    F3Vel = 57,
    F3RPM = 58,
    F3Speed = 59,
    F4Vel = 60,
    F4RPM = 61,
    F4Speed = 62,
    Temp = 63
};

//Lookup table
static const char* FieldNames[64] = {
    "",                 // 0
    "Device ID",        // 1
    "Bus Voltage",      // 2
    "Output Current",   // 3
    "Output Percent",   // 4
    "Temperature",      // 5
    "Sensor Position",  // 6
    "Sensor Velocity",  // 7
    "Max Current",      // 8
    "Temp Disable",     // 9
    "Error",            // 10
    "Voltage",          // 11
    "Motor Number",     // 12
    "Speed",            // 13
    "Potentiometer",    // 14
    "Time Without Change", // 15
    "Max",              // 16
    "Min",              // 17
    "At Min",           // 18
    "At Max",           // 19
    "Distance",         // 20
    "Sensorless",       // 21
    "Robot State",      // 22
    "Excavation State", // 23
    "Error State",      // 24
    "Diagnostics State",// 25
    "Tilt State",       // 26
    "Dump State",       // 27
    "Level Bucket",     // 28
    "Level Arms",       // 29
    "Dest X",           // 30
    "Dest Z",           // 31
    "X",                // 32
    "Y",                // 33
    "Z",                // 34
    "roll",             // 35
    "pitch",            // 36
    "yaw",              // 37
    "aruco",            // 38
    "RSSI",             // 39
    "Wi-Fi",            // 40
    "CAN Bus",          // 41
    "Using Can1",       // 42
    "RX Packets",       // 43
    "TX Packets",       // 44
    "CAN Bus2",         // 45
    "RX2 Packets",      // 46
    "TX2 Packets",      // 47
    "First Motor",      // 48
    "Second Motor",     // 49
    "Num Breaks",       // 50
    "F1 Vel",           // 51
    "F1 RPM",           // 52
    "F1 Speed",         // 53
    "F2 Vel",           // 54
    "F2 RPM",           // 55
    "F2 Speed",         // 56
    "F3 Vel",           // 57
    "F3 RPM",           // 58
    "F3 Speed",         // 59
    "F4 Vel",           // 60
    "F4 RPM",           // 61
    "F4 Speed",         // 62
    "Temp"              // 63
};

struct Element{

    Element(std::string label, std::list<Data> data, uint8_t type);
    Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, ...);
    Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, std::vector<size_t> sizeList);

    Element(Field_Strings field, std::list<Data> data, uint8_t type);
    Element(Field_Strings field, std::list<Data> data, uint8_t type, size_t dimensionCount, ...);
    Element(Field_Strings field, std::list<Data> data, uint8_t type, size_t dimensionCount, std::vector<size_t> sizeList);
    //    Element(std::string label, std::list<Data> data, uint8_t type){
//        this->label = std::move(label);
//        this->type = type;
//        this->dimensionCount = 1;
//        this->sizeList.push_back(1);
//        this->data = std::move(data);
//    }
//    Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, ...){
//        this->label = std::move(label);
//        this->data = std::move(data);
//        this->type = type;
//
//        this->dimensionCount = dimensionCount;
//        va_list va;
//        va_start(va, dimensionCount);
//        for(int index=0; index < dimensionCount; index++){
//            size_t x=va_arg(va, size_t);
//            this->sizeList.push_back(x);
//        }
//        va_end(va);
//    }
//    Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, std::vector<size_t> sizeList){
//        this->label = std::move(label);
//        this->data = std::move(data);
//        this->type = type;
//        this->dimensionCount = dimensionCount;
//        this->sizeList = std::move(sizeList);
//    }
    bool label_is_field = false;
    uint8_t label_field_id = 0;
    std::string label;
    uint8_t type;
    size_t dimensionCount;
    std::vector<size_t> sizeList;
    std::list<Data> data;

    void print();
};

struct Object{
    std::string label;
    uint8_t type;
    std::vector<Element> elementList;
    std::vector<Object> children;

    void print();
};

class BinaryMessage {

    Object topObject;

public:
    BinaryMessage(std::string label);
    BinaryMessage(std::list<uint8_t>& bytes);
    void print();
    void printObject(Object object);
    void printElement(Element element);

    std::string getLabel();
    Object getObject();

    static bool hasSize(std::list<uint8_t>& message);
    static bool hasMessage(std::list<uint8_t>& message);
    static uint64_t decodeSizeBytes(std::list<uint8_t>& bytes);
    uint64_t decodeSizeBytes(std::list<uint8_t>::iterator& currentByte);
    std::string decodeLabel(std::list<uint8_t>::iterator& currentByte);
    Object decodeObject(std::list<uint8_t>::iterator& currentByte);
    Element decodeElement(std::list<uint8_t>::iterator& currentByte);
    uint8_t decodeType(std::list<uint8_t>::iterator& currentByte);

    std::list<uint8_t> encodeSizeBytes(uint64_t number);
    void encodeSizeBytes(std::shared_ptr<std::list<uint8_t>>bytes, uint64_t number);
    void encodeMessageSizeBytes(std::shared_ptr<std::list<uint8_t>> bytes);
    void addElementBoolean(std::string label, bool boolean);
    void addElementCharacter(std::string label, char character);
    void addElementInt8(std::string label, int8_t int8);
    void addElementInt16(std::string label, int16_t int16);
    void addElementInt32(std::string label, int32_t int32);
    void addElementInt64(std::string label, int64_t int64);
    void addElementUInt8(std::string label, uint8_t uint8);
    void addElementUInt16(std::string label, uint16_t uint16);
    void addElementUInt32(std::string label, uint32_t uint32);
    void addElementUInt64(std::string label, uint64_t uint64);
    void addElementFloat32(std::string label, float float32);
    void addElementFloat64(std::string label, double float64);
    void addElementString(std::string label, std::string string);
    void addElementBooleanArray(std::string label, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementCharacterArray(std::string label, std::vector<char> characterList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt8Array(std::string label, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt16Array(std::string label, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt32Array(std::string label, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt64Array(std::string label, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt8Array(std::string label, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt16Array(std::string label, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt32Array(std::string label, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt64Array(std::string label, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat32Array(std::string label, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat64Array(std::string label, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addChild(Object childObject);

    void addElementBoolean(Object& object, std::string label, bool boolean);
    void addElementBoolean(Object& object, Field_Strings field, bool boolean);
    void addElementCharacter(Object& object, std::string label, char character);
    void addElementCharacter(Object& object, Field_Strings field, char character);
    void addElementInt8(Object& object, std::string label, int8_t int8);
    void addElementInt8(Object& object, Field_Strings field, int8_t int8);
    void addElementInt16(Object& object, std::string label, int16_t int16);
    void addElementInt16(Object& object, Field_Strings field, int16_t int16);
    void addElementInt32(Object& object, std::string label, int32_t int32);
    void addElementInt32(Object& object, Field_Strings field, int32_t int32);
    void addElementInt64(Object& object, std::string label, int64_t int64);
    void addElementInt64(Object& object, Field_Strings field, int64_t int64);
    void addElementUInt8(Object& object, std::string label, uint8_t uint8);
    void addElementUInt8(Object& object, Field_Strings field, uint8_t uint8);
    void addElementUInt16(Object& object, std::string label, uint16_t uint16);
    void addElementUInt16(Object& object, Field_Strings field, uint16_t uint16);
    void addElementUInt32(Object& object, std::string label, uint32_t uint32);
    void addElementUInt32(Object& object, Field_Strings field, uint32_t uint32);
    void addElementUInt64(Object& object, std::string label, uint64_t uint64);
    void addElementUInt64(Object& object, Field_Strings field, uint64_t uint64);
    void addElementFloat32(Object& object, std::string label, float float32);
    void addElementFloat32(Object& object, Field_Strings field, float float32);
    void addElementFloat64(Object& object, std::string label, double float64);
    void addElementFloat64(Object& object, Field_Strings field, double float64);
    void addElementString(Object& object, std::string label, std::string string);
    void addElementString(Object& object, Field_Strings field, std::string string);
    void addElementBooleanArray(Object& object, std::string label, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementBooleanArray(Object& object, Field_Strings field, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementCharacterArray(Object& object, std::string label, std::vector<char> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementCharacterArray(Object& object, Field_Strings field, std::vector<char> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt8Array(Object& object, std::string label, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt8Array(Object& object, Field_Strings field, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt16Array(Object& object, std::string label, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt16Array(Object& object, Field_Strings field, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt32Array(Object& object, std::string label, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt32Array(Object& object, Field_Strings field, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt64Array(Object& object, std::string label, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt64Array(Object& object, Field_Strings field, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt8Array(Object& object, std::string label, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt8Array(Object& object, Field_Strings field, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt16Array(Object& object, std::string label, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt16Array(Object& object, Field_Strings field, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt32Array(Object& object, std::string label, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt32Array(Object& object, Field_Strings field, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt64Array(Object& object, std::string label, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt64Array(Object& object, Field_Strings field, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat32Array(Object& object, std::string label, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat32Array(Object& object, Field_Strings field, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat64Array(Object& object, std::string label, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat64Array(Object& object, Field_Strings field, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addChild(Object& object, Object childObject);
    
    //Encoded Elements
    std::string decodeFieldValue(Field_Strings field);
    void addElementBoolean(Field_Strings field, bool boolean);
    void addElementCharacter(Field_Strings field, char character);
    void addElementInt8(Field_Strings field, int8_t int8);
    void addElementInt16(Field_Strings field, int16_t int16);
    void addElementInt32(Field_Strings field, int32_t int32);
    void addElementInt64(Field_Strings field, int64_t int64);
    void addElementUInt8(Field_Strings field, uint8_t uint8);
    void addElementUInt16(Field_Strings field, uint16_t uint16);
    void addElementUInt32(Field_Strings field, uint32_t uint32);
    void addElementUInt64(Field_Strings field, uint64_t uint64);
    void addElementFloat32(Field_Strings field, float float32);
    void addElementFloat64(Field_Strings field, double float64);
    void addElementString(Field_Strings field, std::string string);
    void addElementBooleanArray(Field_Strings field, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementCharacterArray(Field_Strings field, std::vector<char> characterList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt8Array(Field_Strings field, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt16Array(Field_Strings field, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt32Array(Field_Strings field, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt64Array(Field_Strings field, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt8Array(Field_Strings field, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt16Array(Field_Strings field, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt32Array(Field_Strings field, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt64Array(Field_Strings field, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat32Array(Field_Strings field, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat64Array(Field_Strings field, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList);
    
    void encodeBytes(std::shared_ptr<std::list<uint8_t>> bytes, Object object);
    void encodeBytes(std::shared_ptr<std::list<uint8_t>> bytes, Element element);
    void encodeLabelBytes(std::shared_ptr<std::list<uint8_t>> bytes, std::string label);
    void encodeLabelBytes(std::shared_ptr<std::list<uint8_t>> bytes, Field_Strings field);
    void addSizeBytes(std::shared_ptr<std::list<uint8_t>> bytes, uint64_t size);

    std::shared_ptr<std::list<uint8_t>> getBytes();

    //Object getObject();
};
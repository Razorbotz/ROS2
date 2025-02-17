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

struct Element{
    /*
        Three constructor declerations for Element class
        1. Element(std::string label, std::list<Data> data, uint8_t type)
                Purpose: Create a scalar element that holds a single value
                Usage: pass in the label, data, and type of the element
        2. Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, ...) 
                Purpose: To create an element that represents an array with one or more dimensions
                Usage: In addition to the label, data, and type, you specify the number of dimensions (dimensionCount). Then, using variadic arguments (...), you supply each dimension’s size.
        3. Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, std::vector<size_t> sizeList)
                Purpose: To create an element that represents an array with one or more dimensions
                Usage: In addition to the label, data, and type, you specify the number of dimensions (dimensionCount). Then, you supply a vector of size_t values that represent the size of each dimension.
            
    */
    Element(std::string label, std::list<Data> data, uint8_t type);
    Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, ...);
    Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, std::vector<size_t> sizeList);
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

/*
    Data Members
*/
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
    //Connects to the Element struct
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
    void addElementCharacter(Object& object, std::string label, char character);
    void addElementInt8(Object& object, std::string label, int8_t int8);
    void addElementInt16(Object& object, std::string label, int16_t int16);
    void addElementInt32(Object& object, std::string label, int32_t int32);
    void addElementInt64(Object& object, std::string label, int64_t int64);
    void addElementUInt8(Object& object, std::string label, uint8_t uint8);
    void addElementUInt16(Object& object, std::string label, uint16_t uint16);
    void addElementUInt32(Object& object, std::string label, uint32_t uint32);
    void addElementUInt64(Object& object, std::string label, uint64_t uint64);
    void addElementFloat32(Object& object, std::string label, float float32);
    void addElementFloat64(Object& object, std::string label, double float64);
    void addElementString(Object& object, std::string label, std::string string);
    void addElementBooleanArray(Object& object, std::string label, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementCharacterArray(Object& object, std::string label, std::vector<char> booleanList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt8Array(Object& object, std::string label, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt16Array(Object& object, std::string label, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt32Array(Object& object, std::string label, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementInt64Array(Object& object, std::string label, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt8Array(Object& object, std::string label, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt16Array(Object& object, std::string label, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt32Array(Object& object, std::string label, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementUInt64Array(Object& object, std::string label, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat32Array(Object& object, std::string label, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addElementFloat64Array(Object& object, std::string label, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList);
    void addChild(Object& object, Object childObject);

    void encodeBytes(std::shared_ptr<std::list<uint8_t>> bytes, Object object);
    void encodeBytes(std::shared_ptr<std::list<uint8_t>> bytes, Element element);
    void encodeLabelBytes(std::shared_ptr<std::list<uint8_t>> bytes, std::string label);
    void addSizeBytes(std::shared_ptr<std::list<uint8_t>> bytes, uint64_t size);

    std::shared_ptr<std::list<uint8_t>> getBytes();

    //Object getObject();
};



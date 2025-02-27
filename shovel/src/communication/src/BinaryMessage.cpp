#include <list>
#include <iostream>
#include <string>

#include "BinaryMessage.hpp"


Element::Element(std::string label, std::list<Data> data, uint8_t type){
    this->label = std::move(label);
    this->type = type;
    this->dimensionCount = 1;
    this->sizeList.push_back(1);
    this->data = std::move(data);
}


Element::Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, ...){
    this->label = std::move(label);
    this->data = std::move(data);
    this->type = type;

    this->dimensionCount = dimensionCount;
    va_list va;
    va_start(va, dimensionCount);
    for(int index=0; index < dimensionCount; index++){
        size_t x=va_arg(va, size_t);
        this->sizeList.push_back(x);
    }
    va_end(va);
}


Element::Element(std::string label, std::list<Data> data, uint8_t type, size_t dimensionCount, std::vector<size_t> sizeList){
    this->label = std::move(label);
    this->data = std::move(data);
    this->type = type;
    this->dimensionCount = dimensionCount;
    this->sizeList = std::move(sizeList);
}

void Element::print(){
//    std::cout << "Element" << std::endl;
    std::cout << this->label << ": ";

    if(this->type == TYPE::BOOLEAN){
        std::cout << this->data.begin()->boolean << std::endl;
    }
    else if(this->type == TYPE::CHARACTER){
        std::cout << this->data.begin()->character << std::endl;
    }
    else if(this->type == TYPE::INT8){
        std::cout << (int)this->data.begin()->int8 << std::endl;
    }
    else if(this->type == TYPE::INT16){
        std::cout << this->data.begin()->int16 << std::endl;
    }
    else if(this->type == TYPE::INT32){
        std::cout << this->data.begin()->int32 << std::endl;
    }
    else if(this->type == TYPE::INT64){
        std::cout << this->data.begin()->int64 << std::endl;
    }
    else if(this->type == TYPE::UINT8){
        std::cout << (uint32_t)this->data.begin()->uint8 << std::endl;
    }
    else if(this->type == TYPE::UINT16){
        std::cout << this->data.begin()->uint16 << std::endl;
    }
    else if(this->type == TYPE::UINT32){
        std::cout << this->data.begin()->uint32 << std::endl;
    }
    else if(this->type == TYPE::UINT64){
        std::cout << this->data.begin()->uint64 << std::endl;
    }
    else if(this->type == TYPE::FLOAT32){
        std::cout << this->data.begin()->float32 << std::endl;
    }
    else if(this->type == TYPE::FLOAT64){
        std::cout << this->data.begin()->float64 << std::endl;
    }
    else if(this->type == TYPE::STRING){
        std::string string="";
        for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
            string+= iterator->character;
        }
        std::cout << string << std::endl;
    }
    else if(this->type == TYPE::ARRAYBOOLEAN){

        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->boolean << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->boolean << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYCHARACTER){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->character << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->character << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYINT8){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->int8 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->int8 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYINT16){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->int16 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->int16 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYINT32){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->int32 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->int32 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYINT64){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->int64 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->int64 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYUINT8){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->uint8 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->uint8 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYUINT16){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->uint16 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->uint16 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYUINT32){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->uint32 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->uint32 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << "----" << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYUINT64){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->uint64 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->uint64 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYFLOAT32){
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->float32 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->float32 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
    else if(this->type == TYPE::ARRAYFLOAT64) {
        if(this->dimensionCount==1){
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++){
                std::cout << iterator->float64 << " ";
            }
            std::cout << std::endl;
        }
        if(this->dimensionCount==2){
            std::cout << std::endl;
            size_t lineIndex = 0;
            for(std::list<Data>::iterator iterator = this->data.begin(); iterator != this->data.end(); iterator++, lineIndex++){
                std::cout << iterator->float64 << " ";
                if(lineIndex == this->sizeList[1]-1){
                    std::cout << std::endl;
                    lineIndex=-1;
                }
            }
            std::cout << std::endl;
        }
    }
}


void Object::print(){
    std::cout << "Object" << std::endl;
    std::cout << "label: " << this->label << std::endl;
    for (auto element=this->elementList.begin(); element != this->elementList.end(); element++){
        element->print();
        //printElement(*element);
    }
    for (auto object2=this->children.begin(); object2 != this->children.end(); object2++){
        this->print();
        //printObject(*object2);
    }
}

//Constructor called in control.cpp to decode binary message
BinaryMessage::BinaryMessage(std::list<uint8_t>& bytes){
    std::list<uint8_t>::iterator currentByte = bytes.begin();

    uint64_t size = decodeSizeBytes(currentByte);
    this->topObject = decodeObject(currentByte);
}


std::string BinaryMessage::getLabel(){
    return this->topObject.label;
}

Object BinaryMessage::getObject(){
    return this->topObject;
}


void BinaryMessage::print(){
    //printObject(this->topObject);
    this->topObject.print();
}


Object BinaryMessage::decodeObject(std::list<uint8_t>::iterator& currentByte){
    Object object;
    object.label=decodeLabel(currentByte);
    object.type=*currentByte;
    currentByte++;
    if(object.type!=TYPE::OBJECT){
        std::cout << "data not in sync OBJECT" << std::endl;
        std::cout << "Data of type : " << object.type << std::endl;
    }
    else{
        uint64_t elementCount = decodeSizeBytes(currentByte);
        for(int index=0; index < elementCount ; index++){
            Element element = decodeElement(currentByte);
            object.elementList.push_back(element);
        }
        uint64_t objectCount = decodeSizeBytes(currentByte);
        for(int index=0; index < objectCount ; index++){
            Object object = decodeObject(currentByte);
            object.children.push_back(object);
        }
        return object;
    }
}


std::string BinaryMessage::decodeLabel(std::list<uint8_t>::iterator& currentByte){
    int dataType=*currentByte;
    currentByte++;
    if(dataType != TYPE::STRING){
        std::cout << "data not in sync LABEL" << std::endl;
        std::cout << "Label of type: " << dataType << std::endl;
    }
    else{
        uint64_t size=decodeSizeBytes(currentByte);
        std::string label="";
        for(int index=0; index < size ; index++){
            label += *(currentByte);
            currentByte++;
        }
        return label;
    }
}


uint8_t BinaryMessage::decodeType(std::list<uint8_t>::iterator& currentByte){
    uint8_t type = *currentByte;
    currentByte++;
    return type;
}


Element BinaryMessage::decodeElement(std::list<uint8_t>::iterator& currentByte){
    std::string label = decodeLabel(currentByte);
    uint8_t type = decodeType(currentByte);
    size_t dimensionCount=1;
    std::vector<size_t> sizeList;
    std::list<Data> dataList;

    if(type == TYPE::BOOLEAN){
        sizeList.push_back(1);
        Data data;
        data.boolean = *currentByte;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::CHARACTER){
        sizeList.push_back(1);
        Data data;
        data.character = *currentByte;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::INT8){
        sizeList.push_back(1);
        Data data;
        data.int8 = *currentByte;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::INT16){
        sizeList.push_back(1);
        Data data;
        data.int16  = (int16_t)(*currentByte) << 8;
        currentByte++;
        data.int16 |= (int16_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::INT32){
        sizeList.push_back(1);
        Data data;
        data.int32  = (int32_t)(*currentByte) << 24;
        currentByte++;
        data.int32 |= (int32_t)(*currentByte) << 16;
        currentByte++;
        data.int32 |= (int32_t)(*currentByte) << 8;
        currentByte++;
        data.int32 |= (int32_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::INT64){
        sizeList.push_back(1);
        Data data;
        data.int64  = (int64_t)(*currentByte) << 56;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 48;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 40;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 32;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 24;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 16;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 8;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::UINT8){
        sizeList.push_back(1);
        Data data;
        data.uint8 = *currentByte;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::UINT16){
        sizeList.push_back(1);
        Data data;
        data.uint16  = (uint16_t)(*currentByte) << 8;
        currentByte++;
        data.uint16 |= (uint16_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::UINT32){
        sizeList.push_back(1);
        Data data;
        data.uint32  = (uint32_t)(*currentByte) << 24;
        currentByte++;
        data.uint32 |= (uint32_t)(*currentByte) << 16;
        currentByte++;
        data.uint32 |= (uint32_t)(*currentByte) << 8;
        currentByte++;
        data.uint32 |= (uint32_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::UINT64){
        sizeList.push_back(1);
        Data data;
        data.uint64  = ((uint64_t)(*currentByte)) << 56;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 48;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 40;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 32;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 24;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 16;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 8;
        currentByte++;
        data.uint64 |= ((uint64_t)(*currentByte)) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::FLOAT32){
        sizeList.push_back(1);
        Data data;
        data.int32  = (int32_t)(*currentByte) << 24;
        currentByte++;
        data.int32 |= (int32_t)(*currentByte) << 16;
        currentByte++;
        data.int32 |= (int32_t)(*currentByte) << 8;
        currentByte++;
        data.int32 |= (int32_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::FLOAT64){
        sizeList.push_back(1);
        Data data;
        data.int64  = (int64_t)(*currentByte) << 56;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 48;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 40;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 32;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 24;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 16;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 8;
        currentByte++;
        data.int64 |= (int64_t)(*currentByte) << 0;
        currentByte++;
        dataList.push_back(data);
    }
    else if(type == TYPE::STRING){
        size_t size = decodeSizeBytes(currentByte);
        sizeList.push_back(size);

        for(int index = 0; index < size ; index++){
            Data data;
            data.character = *currentByte;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYBOOLEAN){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.boolean = *currentByte;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYCHARACTER){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.character = *currentByte;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYINT8){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.int8 = *currentByte;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYINT16){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.uint16  = (uint16_t)(*currentByte) << 8;
            currentByte++;
            data.uint16 |= (uint16_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYINT32){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.int32  = (int32_t)(*currentByte) << 24;
            currentByte++;
            data.int32 |= (int32_t)(*currentByte) << 16;
            currentByte++;
            data.int32 |= (int32_t)(*currentByte) << 8;
            currentByte++;
            data.int32 |= (int32_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYINT64){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.int64  = (int64_t)(*currentByte) << 56;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 48;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 40;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 32;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 24;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 16;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 8;
            currentByte++;
            data.int64 |= (int64_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYUINT8){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.int8 = *currentByte;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYUINT16){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.uint16  = (uint16_t)(*currentByte) << 8;
            currentByte++;
            data.uint16 |= (uint16_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYUINT32){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.uint32  = (int32_t)(*currentByte) << 24;
            currentByte++;
            data.uint32 |= (int32_t)(*currentByte) << 16;
            currentByte++;
            data.uint32 |= (int32_t)(*currentByte) << 8;
            currentByte++;
            data.uint32 |= (int32_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYUINT64){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.uint64  = (int64_t)(*currentByte) << 56;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 48;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 40;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 32;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 24;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 16;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 8;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYFLOAT32){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.uint32  = (int32_t)(*currentByte) << 24;
            currentByte++;
            data.uint32 |= (int32_t)(*currentByte) << 16;
            currentByte++;
            data.uint32 |= (int32_t)(*currentByte) << 8;
            currentByte++;
            data.uint32 |= (int32_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }
    else if(type == TYPE::ARRAYFLOAT64){
        dimensionCount = decodeSizeBytes(currentByte);
        uint64_t totalValues=1;
        for(int index = 0; index < dimensionCount; index++){
            size_t size = decodeSizeBytes(currentByte);
            sizeList.push_back(size);
            totalValues *= size;
        }
        for(uint64_t index = 0; index < totalValues; index++){
            Data data;
            data.uint64  = (int64_t)(*currentByte) << 56;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 48;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 40;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 32;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 24;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 16;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 8;
            currentByte++;
            data.uint64 |= (int64_t)(*currentByte) << 0;
            currentByte++;
            dataList.push_back(data);
        }
    }

    Element element(label, dataList, type, dimensionCount, sizeList);
    return element;
}


BinaryMessage::BinaryMessage(std::string label){
    this->topObject.label = label;
}


std::list<uint8_t> BinaryMessage::encodeSizeBytes( uint64_t number){
    std::shared_ptr<std::list<uint8_t>> bytes = std::make_shared<std::list<uint8_t>>();

    encodeSizeBytes(bytes, number);

    return *bytes.get();
}

// Seems redundent due to addSizeBytes
void BinaryMessage::encodeSizeBytes(std::shared_ptr<std::list<uint8_t>> bytes, uint64_t number) {

    if (number <= 0x7F) {
        bytes->push_back((uint8_t) number);
    }
    else {
        uint64_t mask = 0xFF00000000000000L;
        int zeroCount = 0;
        for (int index = 0; index < 8; index++) {
            if ((mask & number) == 0) {
                zeroCount++;
            }
            else {
                break;
            }
            mask = mask >> 8;
        }

        int byteCount = 8 - zeroCount;

        // Sets MSB to 1 and writes the number of bytes that are required to output
        bytes->push_back(byteCount | 0x80);

        // Writes the bytes in big-endian order
        for (int index = byteCount - 1; index >= 0; index--) {
            // Shifts the number to the right by 8 bits times the index and then masks it with 0xFF to get the least significant byte
            uint8_t byte = (uint8_t) (number >> (index * 8) & 0xFF);
            // Writes the byte to the output
            bytes->push_back(byte);
        }
    }
}


void BinaryMessage::encodeMessageSizeBytes(std::shared_ptr<std::list<uint8_t>> bytes){

    uint64_t number = bytes->size();

    int adjustment = 0;
    if(number <= 0x7E) {
        adjustment = 1;
    }
    else if(number <= 0xFD){
        adjustment = 2;
    }
    else if(number <= 0xFFFC){
        adjustment = 3;
    }
    else if(number <= 0xFFFFFB){
        adjustment = 4;
    }
    else if(number <= 0xFFFFFFFA){
        adjustment = 5;
    }
    else if(number <= 0xFFFFFFFFF9){
        adjustment = 6;
    }
    else if(number <= 0xFFFFFFFFFFF8){
        adjustment = 7;
    }
    else if(number <= 0xFFFFFFFFFFFFF7){
        adjustment = 8;
    }
    else if(number <= 0xFFFFFFFFFFFFFFF6){
        adjustment = 9;
    }
    number+=adjustment;


    // Do function call for encodeSizeBytes instead
    if(number <= 0x7F){
        bytes->push_front((uint8_t)number);
    }
    else {
        uint64_t mask = 0xFF00000000000000L;
        int zeroCount = 0;
        for (int index = 0; index < 8; index++) {
            if ((mask & number) == 0) {
                zeroCount++;
            }
            else {
                break;
            }
            mask = mask >> 8;
        }

        int byteCount = 8 - zeroCount;

        for (int index = 0; index < byteCount; index++) {
            uint8_t byte = (uint8_t) (number >> (index * 8) & 0xFF);
            bytes->push_front(byte);
        }

        bytes->push_front(byteCount | 0x80);
    }
}


bool BinaryMessage::hasSize(std::list<uint8_t>& message){
    if(message.size()==0){
        return false;
    }
    std::list<uint8_t>::iterator currentByte = message.begin();
    if(*currentByte < 0x80){
        return true;
    }
    else{
        int byteCount = *currentByte & 0x7F;
        if (message.size()>=byteCount){
            return true;
        }
    }
    return false;
}


bool BinaryMessage::hasMessage(std::list<uint8_t>& message){
    if(hasSize(message)){
        long messageSize=BinaryMessage::decodeSizeBytes(message);
        int listSize=message.size();
        if(messageSize<=listSize){
            return true;
        }
        else {
            return false;
        }
    }
    return false;
}


uint64_t BinaryMessage::decodeSizeBytes(std::list<uint8_t>& message){
    int byteCount=0;
    std::list<uint8_t>::iterator currentByte = message.begin();
    if(*currentByte < 0x80){
        return (uint64_t) (*currentByte & 0x7F);
    }
    else{
        byteCount = *currentByte & 0x7F;
    }

    currentByte++;
    long number=0;
    for(int currentByteIndex=0; currentByteIndex < byteCount; currentByteIndex++, currentByte++) {
        number = number << 8;
        number = number | (uint64_t)(*currentByte);
    }

    return number;
}


uint64_t BinaryMessage::decodeSizeBytes(std::list<uint8_t>::iterator& currentByte){
    uint64_t size=0;

    int byteCount=0;
    if(*currentByte < 0x80){
        size = (uint64_t) (*currentByte & 0x7F);
        currentByte++;
    }
    else{
        byteCount = *currentByte & 0x7F;
        currentByte++;
        for(int currentByteIndex=0; currentByteIndex < byteCount; currentByteIndex++) {
            size = size << 8;
            size = size | (uint64_t)(*currentByte);
            currentByte++;
        }
    }

    return size;
}


void BinaryMessage::addElementBoolean(std::string label, bool boolean){
    addElementBoolean(this->topObject,label, boolean);
}


void BinaryMessage::addElementCharacter(std::string label, char character){
    addElementCharacter(this->topObject,label, character);
}


void BinaryMessage::addElementInt8(std::string label, int8_t int8){
    addElementInt8(this->topObject,label, int8);
}


void BinaryMessage::addElementInt16(std::string label, int16_t int16){
    addElementInt16(this->topObject,label, int16);
}


void BinaryMessage::addElementInt32(std::string label, int32_t int32){
    addElementInt32(this->topObject,label, int32);
}


void BinaryMessage::addElementInt64(std::string label, int64_t int64){
    addElementInt64(this->topObject,label, int64);
}


void BinaryMessage::addElementUInt8(std::string label, uint8_t uint8){
    addElementUInt8(this->topObject,label, uint8);
}


void BinaryMessage::addElementUInt16(std::string label, uint16_t uint16){
    addElementUInt16(this->topObject,label, uint16);
}


void BinaryMessage::addElementUInt32(std::string label, uint32_t uint32){
    addElementUInt32(this->topObject,label, uint32);
}


void BinaryMessage::addElementUInt64(std::string label, uint64_t uint64){
    addElementUInt64(this->topObject,label, uint64);
}


void BinaryMessage::addElementFloat32(std::string label, float float32){
    addElementFloat32(this->topObject,label, float32);
}


void BinaryMessage::addElementFloat64(std::string label, double float64){
    addElementFloat64(this->topObject,label, float64);
}


void BinaryMessage::addElementString(std::string label, std::string string){
    addElementString(this->topObject,label, string);
}


void BinaryMessage::addElementBooleanArray(std::string label, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementBooleanArray(this->topObject, label, booleanList, dimensionCount, sizeList);
}


void BinaryMessage::addElementCharacterArray(std::string label, std::vector<char> characterList, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementCharacterArray(this->topObject, label, characterList, dimensionCount, sizeList);
}


void BinaryMessage::addElementInt8Array(std::string label, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementInt8Array(this->topObject, label, int8List, dimensionCount, sizeList);
}


void BinaryMessage::addElementInt16Array(std::string label, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementInt16Array(this->topObject, label, int16List, dimensionCount, sizeList);
}


void BinaryMessage::addElementInt32Array(std::string label, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementInt32Array(this->topObject, label, int32List, dimensionCount, sizeList);
}


void BinaryMessage::addElementInt64Array(std::string label, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementInt64Array(this->topObject, label, int64List, dimensionCount, sizeList);
}


void BinaryMessage::addElementUInt8Array(std::string label, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementUInt8Array(this->topObject, label, uint8List, dimensionCount, sizeList);
}


void BinaryMessage::addElementUInt16Array(std::string label, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementUInt16Array(this->topObject, label, uint16List, dimensionCount, sizeList);
}


void BinaryMessage::addElementUInt32Array(std::string label, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementUInt32Array(this->topObject, label, uint32List, dimensionCount, sizeList);
}


void BinaryMessage::addElementUInt64Array(std::string label, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementUInt64Array(this->topObject, label, uint64List, dimensionCount, sizeList);
}


void BinaryMessage::addElementFloat32Array(std::string label, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementFloat32Array(this->topObject, label, floatList, dimensionCount, sizeList);
}


void BinaryMessage::addElementFloat64Array(std::string label, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList){
    addElementFloat64Array(this->topObject, label, doubleList, dimensionCount, sizeList);
}


void BinaryMessage::addChild(Object childObject){
    addChild(this->topObject, childObject);
}



/*
Example Of Helper function 
void BinaryMessage::addElement**Type**(Object& object, std::string label, type variable){
    // Create a Data object
    Data data;
    //Assign the variable to the Data union Field (See BinaryMessage.hpp Data union)
    data.unionField = variable;

    // The element constructor requires a list of Data objects, even for a single value. 
    // Create a new list and add the Data object to the list
    std::list<Data> list;
    list.push_back(data);

    // Create a new Element object with the label, list of Data objects, and the type of the data
    Element element(label, list, TYPE::BOOLEAN );
    // Add the Element object to the Object's elementList
    object.elementList.push_back(element);
}

*/

/**
 * @brief Add a boolean element to the object
 * 
 * This function 
 * 
 * @param object 
 * @param label 
 * @param boolean 
 */
void BinaryMessage::addElementBoolean(Object& object, std::string label, bool boolean){
    Data data;
    data.boolean = boolean;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::BOOLEAN );
    object.elementList.push_back(element);
}


void BinaryMessage::addElementCharacter(Object& object, std::string label, char character){
    Data data;
    data.character = character;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::CHARACTER );
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt8(Object& object, std::string label, int8_t int8){
    Data data;
    data.int8 = int8;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::INT8);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt16(Object& object, std::string label, int16_t int16){
    Data data;
    data.int16 = int16;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::INT16);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt32(Object& object, std::string label, int32_t int32){
    Data data;
    data.int32 = int32;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::INT32);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt64(Object& object, std::string label, int64_t int64){
    Data data;
    data.int64 = int64;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::INT64);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt8(Object& object, std::string label, uint8_t uint8){
    Data data;
    data.uint8 = uint8;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::UINT8);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt16(Object& object, std::string label, uint16_t uint16){
    Data data;
    data.uint16 = uint16;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::UINT16);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt32(Object& object, std::string label, uint32_t uint32){
    Data data;
    data.uint32 = uint32;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::UINT32);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt64(Object& object, std::string label, uint64_t uint64){
    Data data;
    data.uint64 = uint64;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::UINT64);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementFloat32(Object& object, std::string label, float float32){
    Data data;
    data.float32 = float32;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::FLOAT32);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementFloat64(Object& object, std::string label, double float64){
    Data data={false};
    data.float64 = float64;
    std::list<Data> list;
    list.push_back(data);
    Element element(label, list, TYPE::FLOAT64);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementString(Object& object, std::string label, std::string string){
    std::list<Data> list;

    for(int index=0; index < string.size(); index++){
        Data data={false};
        data.character=string[index];
        list.push_back(data);
    }

    Element element(label,list, TYPE::STRING, 1, string.size());
    object.elementList.push_back(element);
}


void BinaryMessage::addElementBooleanArray(Object& object, std::string label, std::vector<bool> booleanList, size_t dimensionCount, std::vector<size_t> sizeList) {
    std::list<Data> list;
    for (int index = 0; index < booleanList.size(); index++) {
        Data data;
        data.boolean = booleanList[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYBOOLEAN, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementCharacterArray(Object& object, std::string label, std::vector<char> characterList, size_t dimensionCount, std::vector<size_t> sizeList) {
    std::list<Data> list;
    for (int index = 0; index < characterList.size(); index++) {
        Data data;
        data.character = characterList[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYCHARACTER, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt8Array(Object& object, std::string label, std::vector<int8_t> int8List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < int8List.size(); index++) {
        Data data;
        data.int8 = int8List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYINT8, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt16Array(Object& object, std::string label, std::vector<int16_t> int16List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < int16List.size(); index++) {
        Data data;
        data.int16 = int16List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYINT16, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt32Array(Object& object, std::string label, std::vector<int32_t> int32List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < int32List.size(); index++) {
        Data data;
        data.int32 = int32List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYINT32, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementInt64Array(Object& object, std::string label, std::vector<int64_t> int64List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < int64List.size(); index++) {
        Data data;
        data.int8 = int64List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYINT64, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt8Array(Object& object, std::string label, std::vector<uint8_t> uint8List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < uint8List.size(); index++) {
        Data data;
        data.int8 = uint8List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYUINT8, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt16Array(Object& object, std::string label, std::vector<uint16_t> uint16List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < uint16List.size(); index++) {
        Data data;
        data.int8 = uint16List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYUINT16, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt32Array(Object& object, std::string label, std::vector<uint32_t> uint32List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < uint32List.size(); index++) {
        Data data;
        data.int8 = uint32List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYUINT32, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementUInt64Array(Object& object, std::string label, std::vector<uint64_t> uint64List, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < uint64List.size(); index++) {
        Data data;
        data.int8 = uint64List[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYUINT64, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementFloat32Array(Object& object, std::string label, std::vector<float> floatList, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < floatList.size(); index++) {
        Data data;
        data.int8 = floatList[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYFLOAT32, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addElementFloat64Array(Object& object, std::string label, std::vector<double> doubleList, size_t dimensionCount, std::vector<size_t> sizeList){
    std::list<Data> list;
    for (int index = 0; index < doubleList.size(); index++) {
        Data data;
        data.int8 = doubleList[index];
        list.push_back(data);
    }

    Element element(label, list, TYPE::ARRAYFLOAT64, dimensionCount, sizeList);
    object.elementList.push_back(element);
}


void BinaryMessage::addChild(Object& object, Object childObject){
    object.children.push_back(childObject);
}


std::shared_ptr<std::list<uint8_t>> BinaryMessage::getBytes(){
    std::shared_ptr<std::list<uint8_t>> bytes = std::make_shared<std::list<uint8_t>>();

    encodeBytes(bytes, topObject);
    encodeMessageSizeBytes(bytes);

    return bytes;
}


/**
 * @brief Returns a list of bytes that represent the size
 *  
 *   Takes in a size (64 bit integer) and returns a list of bytes that represent the size
 *   Ex. 0xFFFF for the size would push the following onto the bytes list 
 *   [0x82, 0xFF, 0xFF]
 * 
 * @param bytes 
 * @param size 
 */
void BinaryMessage::addSizeBytes(std::shared_ptr<std::list<uint8_t>> bytes, uint64_t size){

    if(size <= 0x7F){
        bytes->push_back((uint8_t)size);
    }
    else {
        uint64_t mask = 0xFF00000000000000L;
        int zeroCount = 0;
        for (int index = 0; index < 8; index++) {
            if ((mask & size) == 0) {
                zeroCount++;
            }
            else {
                break;
            }
            mask = mask >> 8;
        }

        int byteCount = 8 - zeroCount;

        bytes->push_back(byteCount | 0x80);

        for (int index = byteCount - 1; index >= 0; index--) {
            uint8_t byte = (uint8_t) (size >> (index * 8) & 0xFF);
            bytes->push_back(byte);
        }
    }
}


/**
 * @brief Encodes the label of an object or element (strings)
 * 
 * This function encodes the label of an object or element.
 * It first pushes the type of the data (string) onto the bytes list
 * Then it pushes the size of the label onto the bytes list
 * Finally it pushes the label onto the bytes list
 * @param bytes 
 * @param label 
 */
void BinaryMessage::encodeLabelBytes(std::shared_ptr<std::list<uint8_t>> bytes, std::string label){
    // Notifies the decoder that the data will be a string
    bytes->push_back(TYPE::STRING);

    //Gets the size of the label
    addSizeBytes(bytes, label.size() );

    //Encodes the label
    for(int index=0; index < label.size(); index++){
        bytes->push_back(label[index]);
    }
}


/* The Object Struct contains the following
            string label;
            uint8_t type;
            vector<Element> elementList;
            vector<Object> children; 
        This function encodes each of the elements of the Object Struct exept for the type
        Starting with the label, then the elementList, and finally the children

*/



/**
 * @brief Encodes an object struct into a list of bytes
 * 
 *  
 *  It first encodes the label of the object.
 *  Pushes the type of the object onto the bytes list. 
 *  Pushes the size of the elementList onto the bytes list. 
 *  Encodes each element of the elementList.
 *  Pushes the size of the children onto the bytes list.
 *  Finally it encodes each child of the object.
 * @param bytes 
 * @param object 
 */
void BinaryMessage::encodeBytes(std::shared_ptr<std::list<uint8_t>> bytes, Object object) {

    //Encodes the label
    encodeLabelBytes(bytes, object.label);

    //Notifies the decoder that the following bytes are an object
    bytes->push_back(TYPE::OBJECT);

    //Pushes the size of the elementList onto the bytes list
    addSizeBytes(bytes, object.elementList.size());
    //Encodes the elements of the object(Element Struct)
    for (auto iterator = object.elementList.begin();iterator != object.elementList.end(); iterator++) {
        encodeBytes(bytes, *iterator);
    }
    //Pushes the size of the children onto the bytes list
    addSizeBytes(bytes, object.children.size());
    //Encodes the children of the object
    for (auto iterator = object.children.begin();iterator != object.children.end(); iterator++) {
        encodeBytes(bytes, *iterator);
    }
}



/**
 * @brief Encodes an element struct into a list of bytes
 * 
 * This function encodes an element struct into a list of bytes
 * It first encodes the label of the element. 
 * Pushes the type of the element onto the bytes list. 
 * Finally checks the type of the element and encodes the data accordingly
 * 
 * 
 * @param bytes 
 * @param element 
 */
void BinaryMessage::encodeBytes(std::shared_ptr<std::list<uint8_t>> bytes, Element element){
    encodeLabelBytes(bytes, element.label);
    bytes->push_back(element.type);

    /*NOTE: element.data is a list of Data unions (see BinaryMessage.hpp Data union)
            element.data.begin() returns an iterator to the first element in the list
            element.data.begin()-> *Data Type* accesses the *Data Type* field within the union and returns the value
    
    */
    uint8_t type = element.type;

    switch (type)
    {   
        case TYPE::BOOLEAN:

            bytes->push_back(element.data.begin()->boolean);
            break;

        case TYPE::CHARACTER:

            bytes->push_back(element.data.begin()->character);
            break;

        case TYPE::INT8:
        case TYPE::UINT8:

            bytes->push_back(element.data.begin()->int8);
            break;

        case TYPE::INT16:
        case TYPE::UINT16:

            uint16_t value = element.data.begin()->int16;
                uint8_t array[2] = {
                    static_cast<uint_fast8_t>(value >> 8),//MSB
                    static_cast<uint_fast8_t>(value & 0xFF) //LSB 
                };
                //             Where        First    Last
                bytes->insert(bytes->end(), array, array + 2);


            break;
        
        case TYPE::INT32:
        case TYPE::UINT32:
        case TYPE::FLOAT32:

            uint32_t value = element.data.begin()->int32;
            // Breaks the value stored in int32 into four bytes and inserts the array to the bytes list
            uint8_t array[4] = {
                static_cast<uint8_t>(value >> 24),//MSB
                static_cast<uint8_t>(value >> 16),
                static_cast<uint8_t>(value >> 8),
                static_cast<uint8_t>(value & 0xFF)//LSB

            };
            //             Where        First    Last
            bytes->insert(bytes->end(), array, array + 4);

            break;
        
        case TYPE::INT64:
        case TYPE::UINT64:
        case TYPE::FLOAT64:

            uint16_t value = element.data.begin()->int64;
            // Breaks the value stored in int64 into eight bytes and inserts the array to the bytes list
            uint8_t array[8]{
                static_cast<uint8_t>(value >> 56),//MSB
                static_cast<uint8_t>(value >> 48),
                static_cast<uint8_t>(value >> 32),
                static_cast<uint8_t>(value >> 24),  
                static_cast<uint8_t>(value >> 16),
                static_cast<uint8_t>(value >> 8),
                static_cast<uint8_t>(value & 0xFF)//LSB
            };
            //             Where        First    Last
            bytes->insert(bytes->end(), array, array + 8);

            break;
        
        case TYPE::STRING:

            addSizeBytes(bytes, element.sizeList[0]);
            for(std::list<Data>::iterator iterator=element.data.begin(); iterator != element.data.end(); iterator++){
                bytes->push_back(iterator->character);
            }

            break;

        case TYPE::ARRAYBOOLEAN:

            addSizeBytes(bytes, element.dimensionCount);
            for(int index=0; index < element.sizeList.size(); index++){
                addSizeBytes(bytes, element.sizeList[index]);
            }
            for(auto iterator=element.data.begin(); iterator != element.data.end(); iterator++){
                bytes->push_back(iterator->boolean);
            }



            break;

        case TYPE::ARRAYCHARACTER:

            addSizeBytes(bytes, element.dimensionCount);
            for(int index=0; index < element.sizeList.size(); index++){
                addSizeBytes(bytes, element.sizeList[index]);
            }
            for(auto iterator=element.data.begin(); iterator != element.data.end(); iterator++){
                bytes->push_back(iterator->character);
            }

            break;

        case TYPE::ARRAYINT8:
        case TYPE::ARRAYUINT8:

            addSizeBytes(bytes, element.dimensionCount);
            for(int index=0; index < element.sizeList.size(); index++){
                addSizeBytes(bytes, element.sizeList[index]);
            }
            for(auto iterator=element.data.begin(); iterator != element.data.end(); iterator++){
                bytes->push_back(iterator->int8);
            }            

            break; 

        case TYPE::ARRAYINT16:
        case TYPE::ARRAYUINT16:

            addSizeBytes(bytes, element.dimensionCount);
            for(int index=0; index < element.sizeList.size(); index++){
                addSizeBytes(bytes, element.sizeList[index]);
            }
            for(auto iterator=element.data.begin(); iterator != element.data.end(); iterator++){

                // Breaks the value stored in int16 into two bytes and inserts the array to the bytes list
                uint_fast16_t value = iterator->int16;
                uint_fast8_t array[2] = {
                    static_cast<uint_fast8_t>(value >> 8),//MSB
                    static_cast<uint_fast8_t>(value & 0xFF) //LSB 
                };
                //             Where        First    Last
                bytes->insert(bytes->end(), array, array + 2);

            }



            break;

        case TYPE::ARRAYINT32:
        case TYPE::ARRAYUINT32:
        case TYPE::ARRAYFLOAT32:

            addSizeBytes(bytes, element.dimensionCount);
            for(int index=0; index < element.sizeList.size(); index++){
                addSizeBytes(bytes, element.sizeList[index]);
            }
            for(auto iterator=element.data.begin(); iterator != element.data.end(); iterator++){

                // Breaks the value stored in int32 into four bytes and inserts the array to the bytes list
                uint32_t value = iterator->int32;
                uint8_t array[4] = {
                    static_cast<uint8_t>(value >> 24),//MSB
                    static_cast<uint8_t>(value >> 16),
                    static_cast<uint8_t>(value >> 8),
                    static_cast<uint8_t>(value & 0xFF)//LSB

                };
                //             Where        First    Last
                bytes->insert(bytes->end(), array, array + 4);

            }

            break;

        case TYPE::ARRAYINT64:
        case TYPE::ARRAYUINT64:
        case TYPE::ARRAYFLOAT64:

            addSizeBytes(bytes, element.dimensionCount);
            for(int index=0; index < element.sizeList.size(); index++){
                addSizeBytes(bytes, element.sizeList[index]);
            }
            for(auto iterator=element.data.begin(); iterator != element.data.end(); iterator++){
                // Breaks the value stored in int64 into eight bytes and inserts the array to the bytes list
                uint64_t value = iterator->int32;
                uint8_t array[8]{
                    static_cast<uint8_t>(value >> 56),//MSB
                    static_cast<uint8_t>(value >> 48),
                    static_cast<uint8_t>(value >> 32),
                    static_cast<uint8_t>(value >> 24),  
                    static_cast<uint8_t>(value >> 16),
                    static_cast<uint8_t>(value >> 8),
                    static_cast<uint8_t>(value & 0xFF)//LSB
                };
                //             Where        First    Last
                bytes->insert(bytes->end(), array, array + 8);
            }

            break; 

        
        
        
    
    default:
        break;
    }

}

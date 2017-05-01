//
// Created by coumarc9 on 4/28/17.
//

#ifndef PROC_MAPPING_HYDROPHONEMAPPER_H
#define PROC_MAPPING_HYDROPHONEMAPPER_H

#include "proc_mapping/general/BaseObjectMapperInterface.h"

namespace proc_mapping {

    class HydrophoneMapper : public BaseObjectMapperInterface
    {
    public:
        void GetMapObject(MapObjectVector &list) override;

        void ResetMapper() override;

    };

}

#endif //PROC_MAPPING_HYDROPHONEMAPPER_H
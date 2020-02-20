// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_PlaneRpcLibServer_hpp
#define air_PlaneRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/plane/api/PlaneApiBase.hpp"


namespace msr { namespace airlib {

class PlaneRpcLibServer : public RpcLibServerBase {
public:
    PlaneRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
    virtual ~PlaneRpcLibServer();

protected:
    virtual PlaneApiBase* getVehicleApi(const std::string& vehicle_name) override
    {
        return static_cast<PlaneApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
    }

};

}} //namespace
#endif

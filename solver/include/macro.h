#ifndef SOLVER_SLAM_MACRO_H
#define SOLVER_SLAM_MACRO_H

#include <memory>

#define POINTER_TYPEDEFS(TypeName)  \
    typedef std::shared_ptr<TypeName> Ptr; \
    typedef std::shared_ptr<const TypeName> ConstPtr; \
    typedef std::unique_ptr<TypeName> UniquePtr; \
    typedef std::weak_ptr<TypeName> WeakPrt; \
    typedef std::weak_ptr<const TypeName> WeakConstPtr; \
    void definePointerTypedefs##__FILE__##__LINE__(void)
#endif //SOLVER_SLAM_MACRO_HPP

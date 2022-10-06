/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Monday, October 02, 2017 - 13:31:17
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#pragma once

#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/datastructures/image/imageram.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <lablic/lablicmoduledefine.h>
#include <labutils/scalarvectorfield.h>
#include <labutils/rgbaimage.h>
#include <inviwo/core/properties/transferfunctionproperty.h>

namespace inviwo {

/** \docpage{org.inviwo.LICProcessor, LICProcessor}
    ![](org.inviwo.LICProcessor.png?classIdentifier=org.inviwo.LICProcessor)

    Line Integral Convolution with a box kernel.

    ### Inports
      * __vectorField__ 2-dimensional vector field (with vectors of
      two components thus two values within each voxel)
      This processor deals with 2-dimensional data only, therefore it is assumed
      the z-dimension will have size 1 otherwise the 0th slice of the volume
      will be processed.
      * __texture__ Texture to be convolved along the streamlines.

    ### Outports
      * __image__ The image resulting from smearing the given texture
      the streamlines of the given vector field.
*/
class IVW_MODULE_LABLIC_API LICProcessor : public Processor {
    // Friends
    // Types
public:
    // Construction / Deconstruction
public:
    LICProcessor();
    virtual ~LICProcessor() = default;

    // Methods
public:
    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

protected:
    /// Our main computation function
    virtual void process() override;

    // (TODO: Helper functions can be defined here and then implemented in the .cpp)
    // e.g. something like a function for standardLIC, fastLIC, autoContrast, ...
    void LIC(double stepSize, const VectorField2& vectorField,
             const RGBAImage& texture, RGBAImage& licImage);
    void fastLIC(double stepSize, std::vector<std::vector<int>>& visited, const VectorField2& vectorField, const RGBAImage& texture, RGBAImage& licImage);
    void enhanceLIC(double targetMean, double targetSD, RGBAImage& licImage);
    dvec2 pixelToPos(size2_t pixel);
    size2_t posToPixel(dvec2 pos);
    dvec2 getMinMax(const VectorField2& vectorField);
    void colorLIC(const VectorField2& vectorField, RGBAImage& licImage);
    ivec3 HSVtoRGB(double H, double S, double V);
    dvec3 RGBtoHSV(int R, int G, int B);
    // Ports
public:
    // Input vector field
    VolumeInport volumeIn_;

    // Input texture
    ImageInport noiseTexIn_;

    // Output image
    ImageOutport licOut_;

    // Properties
public:
    // TODO: Declare properties
    // IntProperty prop1;
    // BoolProperty prop2;
    IntProperty propKernelSize;
    IntProperty propNSteps;
    BoolProperty propFastLIC;
    BoolProperty propEnhance;
    DoubleProperty propMean;
    DoubleProperty propSD;
    BoolProperty propPaint;
    TransferFunctionProperty propColor;

    // Attributes
private:
    size3_t vectorFieldDims_;
    size2_t texDims_;

    dvec2 bboxMin;
    dvec2 bboxMax;
    dvec2 pixelSize;
};

}  // namespace inviwo

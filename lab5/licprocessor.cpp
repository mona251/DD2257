/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Monday, October 02, 2017 - 13:31:17
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/datastructures/volume/volumeram.h>
#include <lablic/licprocessor.h>
#include <labstreamlines/integrator.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo LICProcessor::processorInfo_{
    "org.inviwo.LICProcessor",  // Class identifier
    "LICProcessor",             // Display name
    "KTH Labs",                 // Category
    CodeState::Experimental,    // Code state
    Tags::None,                 // Tags
};

const ProcessorInfo LICProcessor::getProcessorInfo() const { return processorInfo_; }

LICProcessor::LICProcessor()
    : Processor()
    , volumeIn_("volIn")
    , noiseTexIn_("noiseTexIn")
    , licOut_("licOut")

    , propKernelSize("kernelSize", "Choose Kernel Size (One Diretion)", 50, 1, 100, 10)
    , propFastLIC("fastLIC", "Use Fast LIC", true)
    , propEnhance("enhance", "Enhance Contrast", true)
    , propMean("mean", "Desired Mean", 127, 0, 255, 1)
    , propSD("sd", "Desired Standard Deviation", 26, 0, 255, 1)
    , propNSteps("maxFlic", "N Steps", 1500, 10, 10000, 1)
    , propPaint("paint", "Visualize Vector Field with Color", false)
    , propColor("color", "Colors")
    , propColorFraction("fraction", "Fraction of Color to LIC Gray", 0.5, 0.0, 1.0, 0.1)
// TODO: Register additional properties
{
    // Register ports
    addPort(volumeIn_);
    addPort(noiseTexIn_);
    addPort(licOut_);

    // Register properties
    // TODO: Register additional properties
    addProperties(propKernelSize, propFastLIC, propEnhance, propMean, propSD, propNSteps, propPaint, propColor, propColorFraction);
    
    propEnhance.onChange([this]() {
        if (propEnhance.get()) {
            util::show(propMean, propSD);
        } else {
            util::hide(propMean, propSD);
        }
    });
    
    util::hide(propColor, propColorFraction);
    
    propPaint.onChange([this]() {
        if (propPaint.get()) {
            util::show(propColor, propColorFraction);
        } else {
            util::hide(propColor, propColorFraction);
        }
    });
    
    propColor.get().clear();
    propColor.get().add(0.0f, vec4(0.0f, 0.0f, 1.0f, 1.0f));
    propColor.get().add(1.0f, vec4(1.0f, 0.0f, 0.0f, 1.0f));
    propColor.setCurrentStateAsDefault();
}

void LICProcessor::process() {
    // Get input
    if (!volumeIn_.hasData()) {
        return;
    }

    if (!noiseTexIn_.hasData()) {
        return;
    }

    auto vol = volumeIn_.getData();
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);
    vectorFieldDims_ = vol->getDimensions();

    auto tex = noiseTexIn_.getData();
    const RGBAImage texture = RGBAImage::createFromImage(tex);
    texDims_ = tex->getDimensions();

    double value = texture.readPixelGrayScale(size2_t(0, 0));

    LogProcessorInfo(value);

    // Prepare the output, it has the same dimensions as the texture and rgba values in [0,255]
    auto outImage = std::make_shared<Image>(texDims_, DataVec4UInt8::get());
    RGBAImage licImage(outImage);

    std::vector<std::vector<double>> licTexture(texDims_.x, std::vector<double>(texDims_.y, 0.0));

    // Hint: Output an image showing which pixels you have visited for debugging
    std::vector<std::vector<int>> visited(texDims_.x, std::vector<int>(texDims_.y, 0));

    // TODO: Implement LIC and FastLIC
    // This code instead sets all pixels to the same gray value
    bboxMin = vectorField.getBBoxMin();
    bboxMax = vectorField.getBBoxMax();
    pixelSize = dvec2((bboxMax[0] - bboxMin[0]) / (double)texDims_[0],
                      (bboxMax[1] - bboxMin[1]) / (double)texDims_[1]);
    double stepSize = std::min(pixelSize[0], pixelSize[1]);
    
    if (propFastLIC.get()) {
        fastLIC(stepSize, visited, vectorField, texture, licImage);
    } else {
        LIC(stepSize, vectorField, texture, licImage);
    }

    if (propEnhance.get()) enhanceLIC(propMean.get(), propSD.get(), licImage);

    if (propPaint.get()) colorLIC(vectorField, licImage);

    licOut_.setData(outImage);
}
void LICProcessor::LIC(double stepSize, const VectorField2& vectorField,
                       const RGBAImage& texture, RGBAImage& licImage){
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            dvec2 position = pixelToPos(size2_t(i, j));
            std::vector<dvec2> streamline = Integrator::Streamline(vectorField, position, stepSize, propKernelSize.get());
            int gray = 0;
            //LIC with box kernel
            for (int k = 0; k<streamline.size(); k++) {
                size2_t pixel = posToPixel(streamline[k]);
                gray += texture.readPixelGrayScale(pixel);
            }
            if(streamline.size() != 0){
                gray /= streamline.size();
            }
            licImage.setPixelGrayScale(size2_t(i, j), gray);
        }
    }
}
void LICProcessor::fastLIC(double stepSize, std::vector<std::vector<int>>& visited,
                           const VectorField2& vectorField, const RGBAImage& texture, RGBAImage& licImage) {
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            if (visited[i][j] != 0) { //Skip already visited
                continue;
            }
            dvec2 position = pixelToPos(size2_t(i, j));
            
            //Generate a full streamline
            std::vector<dvec2> streamline = Integrator::Streamline(vectorField, position, stepSize, propNSteps.get());
            
            int gray = 0;
            int currentSize = 0; //Keeps track of kernel size
            
            //Add all middle values to the kernel once at the start
            for(int k = 0; k < propKernelSize.get(); k++){
                if(k == streamline.size()-1){
                    break;
                }
                gray += texture.readPixelGrayScale(posToPixel(streamline[k]));
                currentSize++;
            }
            
            //Keeps track of what to add and what to drop from the kernel
            int hi = propKernelSize.get()-1;
            int low = -propKernelSize.get()-2;
            
            //Visit all the pixels in the streamline
            for(int k = 0; k < streamline.size(); k++){
                low++;
                hi++;
                
                size2_t pixel = posToPixel(streamline[k]);
                
                //Add next element to the kernel
                if(hi < streamline.size()){
                    gray += texture.readPixelGrayScale(posToPixel(streamline[hi]));
                    currentSize++;
                }
                //Remove oldest element from kernel
                if(low >= 0){
                    gray -= texture.readPixelGrayScale(posToPixel(streamline[low]));
                    currentSize--;
                }
                
                //Averages the values if the pixel was already visited
                if(visited[pixel[0]][pixel[1]] == 0){
                    licImage.setPixelGrayScale(pixel, gray/(double)currentSize);
                }
                else{
                    int col = licImage.readPixelGrayScale(pixel)*visited[pixel[0]][pixel[1]]+gray/(double)currentSize;
                    licImage.setPixelGrayScale(pixel, col/((double)visited[pixel[0]][pixel[1]]+1.0));
                }
                //Keeps track of how many times we've visited
                visited[pixel[0]][pixel[1]]+=1;
            }
        }
    }
}
void LICProcessor::enhanceLIC(double targetMean, double targetSD, RGBAImage& licImage) {
    double mean = 0.0;
    double sumSquare = 0.0;
    double n = 0.0;
    //Calculate the mean and SD
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            int gray = licImage.readPixelGrayScale(size2_t(i, j));
            if (gray == 0){
                continue;
            }
            mean += gray;
            sumSquare += pow(gray, 2);
            n++;
        }
    }
    mean /= n;
    double sd = sqrt((sumSquare - n * pow(mean, 2)) / (n - 1));
    
    //Set stretch factor
    double stretch = std::min(targetSD / sd, 10.0);
    
    //Apply to all non black pixels
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            int gray = licImage.readPixelGrayScale(size2_t(i, j));
            if (gray == 0){
                continue;
            }
            gray = int(targetMean + stretch * (gray - mean));
            licImage.setPixelGrayScale(size2_t(i, j), gray);
        }
    }
}
void LICProcessor::colorLIC(const VectorField2& vectorField, RGBAImage& licImage) {
    // Add color to image acording to vector field magnitude and user defined colormap
    // Final color is averaged between the LIC gray and the associated color
    dvec2 minMax = getMinMax(vectorField);
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            dvec2 pos = pixelToPos(size2_t(i, j));
            double mag = glm::length(vectorField.interpolate(pos));
            double interpolate = (mag - minMax[0]) / (minMax[1] - minMax[0]);
            vec4 color = propColor.get().sample(interpolate);
            vec4 gray = licImage.readPixel(size2_t(i, j));
            vec4 finalColor = color * 255.0 * propColorFraction.get() + gray * (1 - propColorFraction.get());
            finalColor[3] = 255.0;
            licImage.setPixel(size2_t(i, j), finalColor);
        }
    }
}
dvec2 LICProcessor::getMinMax(const VectorField2& vectorField) {
    // Calculate the min and max magnitudes of the given vector field
    dvec2 minMax = dvec2(INFINITY, 0);
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            dvec2 pos = pixelToPos(size2_t(i, j));
            double mag = glm::length(vectorField.interpolate(pos));
            minMax[0] = std::min(minMax[0], mag);
            minMax[1] = std::max(minMax[1], mag);
        }
    }
    return minMax;
}
dvec2 LICProcessor::pixelToPos(size2_t pixel) {
    return dvec2(bboxMin[0] + ((double)pixel[0] + 0.5) * pixelSize[0],
                 bboxMin[1] + ((double)pixel[1] + 0.5) * pixelSize[1]);
}
size2_t LICProcessor::posToPixel(dvec2 pos) {
    return size2_t(std::min((int)floor((pos[0] - bboxMin[0]) / pixelSize[0]), (int)texDims_[0] - 1),
                   std::min((int)floor((pos[1] - bboxMin[1]) / pixelSize[1]), (int)texDims_[1] - 1));
}
}  // namespace inviwo

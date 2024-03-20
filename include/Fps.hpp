#ifndef FPS_H
#define FPS_H
#include <SFML/Graphics.hpp>

class FPS
{
    public:
        FPS() : mFrame(0), mFps(0) {}
        const unsigned int getFPS() const { return mFps; }
        void update(){
		if(mClock.getElapsedTime().asSeconds() >= 1.f)
		{
			mFps = mFrame;
			mFrame = 0;
			mClock.restart();
		}

		++mFrame;
	}

    protected:

    private:
        unsigned int mFrame;
        unsigned int mFps;
        sf::Clock mClock;
};

#endif // FPS_H

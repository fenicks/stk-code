
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004-2005 Steve Baker <sjbaker1@airmail.net>
//  Copyright (C) 2006-2007 Eduardo Hernandez Munoz
//  Copyright (C) 2010      Joerg Henrichs
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#ifndef HEADER_BATTLE_AI_HPP
#define HEADER_BATTLE_AI__HPP

#include "karts/controller/ai_base_controller.hpp"
#include "race/race_manager.hpp"
#include "tracks/battle_graph.hpp"
#include "utils/random_generator.hpp"

class AIProperties;
class ThreeStrikesBattle;
class BattleGraph;
class Track;
class Vec3;

namespace irr
{
    namespace scene
    {
        class ISceneNode;
    }
}

class BattleAI : public AIBaseController
{


protected:

    /** Keep a pointer to world. */
    ThreeStrikesBattle *m_world;

public:
                BattleAI(AbstractKart *kart, 
                    StateManager::ActivePlayer *player=NULL);
                //~BattleAI();
                virtual void update      (float delta) {};
    virtual void reset       () {};
    //static void enableDebug() {m_ai_debug = true; }
    virtual void crashed(const AbstractKart *k) {};
    virtual void crashed(const Material *m) {};
    virtual void handleZipper(bool play_sound) {};
    virtual void finishedRace(float time) {};
    virtual void collectedItem(const Item &item, int add_info=-1,
		                       float previous_energy=0) {};
    virtual void setPosition(int p) {};
    virtual bool isNetworkController() const { return false; }
    virtual bool isPlayerController() const { return false; }
    virtual void action(PlayerAction action, int value) {};
    virtual void  skidBonusTriggered() {};
    virtual bool  disableSlipstreamBonus() const {return 0;}
    virtual void  newLap(int lap) {};
};

#endif
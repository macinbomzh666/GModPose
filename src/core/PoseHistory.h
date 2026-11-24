#pragma once

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace GModPose
{
    enum class HistoryAction { Undo, Redo };

    template <class State, class Equal = std::equal_to<State>>
    class PoseHistory
    {
    public:
        static constexpr std::size_t limit = 64;

        void Commit(State state)
        {
            // Reopening on the current pose preserves the existing redo branch.
            if (!states.empty() && Equal{}(states[cursor], state)) return;
            if (!states.empty()) states.resize(cursor + 1);
            states.push_back(std::move(state));
            if (states.size() > limit) states.erase(states.begin());
            cursor = states.size() - 1;
            dirty = true;
        }

        bool CanMove(HistoryAction action) const
        {
            return !states.empty() && (action == HistoryAction::Undo ? cursor > 0 : cursor + 1 < states.size());
        }

        const State* Move(HistoryAction action)
        {
            if (!CanMove(action)) return nullptr;
            if (action == HistoryAction::Redo) ++cursor;
            else --cursor;
            dirty = true;
            return &states[cursor];
        }

        bool Restore(std::vector<State> entries, std::size_t position)
        {
            if (entries.empty() || entries.size() > limit || position >= entries.size()) return false;
            states.clear();
            states.reserve(entries.size());
            // Older files can contain duplicate neighbors. Map the cursor as those collapse.
            for (std::size_t i = 0; i < entries.size(); ++i) {
                if (states.empty() || !Equal{}(states.back(), entries[i]))
                    states.push_back(std::move(entries[i]));
                if (i == position) cursor = states.size() - 1;
            }
            dirty = states.size() != entries.size();
            return true;
        }

        const std::vector<State>& States() const { return states; }
        std::size_t Cursor() const { return cursor; }
        bool Dirty() const { return dirty; }
        void MarkSaved() { dirty = false; }

    private:
        std::vector<State> states;
        std::size_t cursor{};
        bool dirty{};
    };
}
